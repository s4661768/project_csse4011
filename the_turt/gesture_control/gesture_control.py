# Local costmap inflation radius 0.5
# Global costmap inflation radius 0.25
import paho.mqtt.client as mqtt

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg  import OccupancyGrid
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from nav2_msgs.srv import GetCostmap
from nav2_msgs.msg import Costmap
from nav2_msgs.msg import BehaviorTreeLog

from nav2_simple_commander.robot_navigator import BasicNavigator

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient

from enum import Enum
import math
import tf_transformations as tft
import os

import time
import threading
import json

def run_with_timeout(node, timeout_sec):
    end_time = node.get_clock().now() + rclpy.duration.Duration(seconds=timeout_sec)

    while node.get_clock().now() < end_time:
        rclpy.spin_once(node, timeout_sec=timeout_sec)






class OccupancyGrid2d():
    '''
    2D Occupancy Grid Class
    '''
    class CostValues(Enum):
        FreeSpace = 0
        LethalObstacle = 100
        NoInformation = -1

    def __init__(self, map):
        self.map = map

    def __str__(self):
        return f'OccupancyGrid2d: size=({self.getSizeX()}, {self.getSizeY()})'

    def getCost(self, mx, my):
        return self.map.data[self.__getIndex(mx, my)]

    def getSize(self):
        return (self.map.info.width, self.map.info.height)

    def getSizeX(self):
        return self.map.info.width

    def getSizeY(self):
        return self.map.info.height

    def mapToWorld(self, mx, my):
        wx = self.map.info.origin.position.x + (mx + 0.5) * self.map.info.resolution
        wy = self.map.info.origin.position.y + (my + 0.5) * self.map.info.resolution

        return (wx, wy)

    def worldToMap(self, wx, wy):
        if (wx < self.map.info.origin.position.x or wy < self.map.info.origin.position.y):
            raise Exception("World coordinates out of bounds")

        mx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)
        
        if  (my > self.map.info.height or mx > self.map.info.width):
            raise Exception("Out of bounds")

        return (mx, my)

    def __getIndex(self, mx, my):
        return my * self.map.info.width + mx

class Gestcont(Node):
    '''
    gestcont class
    '''
    def __init__(self):



        super().__init__('gestcont')

      
        # MQTT STUFF
        self.mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqttc.on_connect = self.on_connect
        self.mqttc.on_message = self.on_message
        self.mqttc.on_subscribe = self.on_subscribe
        self.mqttc.on_unsubscribe = self.on_unsubscribe
        self.mqttc.user_data_set([])
        self.mqttc.connect("csse4011-iot.zones.eait.uq.edu.au")
    
        #STATUS STUFF
        self.last_pos_x = 0
        self.last_pos_y = 0
        self.last_pos_z = 0
        self.last_mot_x = 0
        self.last_mot_z = 0


        self.odometry_data = None
        

        #Gesture control
        self.last_rec_command = [0,0,0,0,0]





        # Set the log level to INFO
        self.get_logger().set_level(LoggingSeverity.INFO)

        # Set up subscriptions
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
            )
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
       # self.odometry_data = None


        self.bt_subscription = self.create_subscription (
            BehaviorTreeLog,
            'behavior_tree_log',
            self.bt_callback,
            10
        )

        # prevent unused variable warning
        self.lidar_subscription
        self.map_subscription  
        self.odom_subscription
        self.bt_subscription

        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Set up publishers
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'initialpose', 
            10
        )

        # Define class variables
        self.map = None

        self.lidar = None
        self.initial_pose_received = False
        self.updatedMap = False
        self.updatedPose = False
        self.noFrontierCheck = 0
        self.noLocationCheck = 0
        self.location = None
        self.prevLocation = None
        self.btReady = False
        self.visitedWaypoints = []      

        ignoreOrientation = "ros2 param set /planner_server GridBased.use_final_approach_orientation True"
        # Set distance for lethal cost from obstacle
        globalCostRadius = "ros2 param set /global_costmap/global_costmap inflation_layer.inflation_radius 0.25"
        localCostRadius = "ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 0.5"
        # Set closest distance from an obstacle robot can plan path
        globalObstacleRange = "ros2 param set /global_costmap/global_costmap obstacle_layer.scan.obstacle_max_range 1.0"
        localObstacleRange = "ros2 param set /local_costmap/local_costmap voxel_layer.scan.obstacle_max_range 1.0"
        globalRaytraceRange = "ros2 param set /global_costmap/global_costmap obstacle_layer.scan.raytrace_max_range 1.0"
        localRaytraceRange = "ros2 param set /local_costmap/local_costmap voxel_layer.scan.raytrace_max_range 1.0"
        # Set scaling factor for the obstacle cost dropoff (larger means bot can travel closer to obstacles)
        globalCostScaling = "ros2 param set /global_costmap/global_costmap inflation_layer.cost_scaling_factor 1000000000.0"
        localCostScaling = "ros2 param set /local_costmap/local_costmap inflation_layer.cost_scaling_factor 1000000000.0"
        
        os.system(ignoreOrientation)
        os.system(globalCostRadius)
        os.system(localCostRadius)
        os.system(globalObstacleRange)
        os.system(localObstacleRange)
        os.system(globalRaytraceRange)
        os.system(localRaytraceRange)
        os.system(globalCostScaling)
        os.system(localCostScaling)

        self.setInitialPose([-2.0, 0.5])

        # Wait for initial map
        while self.map == None:
            rclpy.spin_once(self, timeout_sec=1.0)
        
        
        #set up the mqtt
        self.mqtt_thread = threading.Thread(target=self.mqtt_process,daemon=True).start()


        # Start controller
        self.gesture_control_protocol()




    ###############################################################################     GESTURE CONTROLLER
    # def gesture_control_protocol(self):

    #     while(1):
    #         print(self.last_rec_command)
            
    #         self.control_move(int(self.last_rec_command))
    #         self.print_position_and_motion()
    #         time.sleep(2)


    def gesture_control_protocol(self):
        while True:
            print(self.last_rec_command)
#            self.control_move(int(self.last_rec_command))
            self.control_move(self.last_rec_command)
            self.print_position_and_motion()
            rclpy.spin_once(self, timeout_sec=1.0)  # Process messages continuously
            time.sleep(2)


    def control_move(self, command):
        twist_msg = Twist()
        if command[0] == 1:
            twist_msg.angular.z = -0.5  # Turn right
        elif command[1] == 1:
            twist_msg.angular.z = 0.5 
        elif command[2] == 1:
            twist_msg.linear.x = 0.4
        elif command[3] == 1:
            twist_msg.linear.x = -0.4

        else:
        # if command >= 0 and command <= 50:
        #     twist_msg.angular.z = -0.5  # Turn right
        # elif command >= 51 and command <= 100:
        #     twist_msg.angular.z = 0.5   # Turn left
        # elif command >= 101 and command <= 150:
        #     twist_msg.linear.x = 0.4    # Move forward
        # elif command >= 151 and command <= 200:
        #     twist_msg.linear.x = -0.4   # Move backward
        # if command >= 0 and command <= 70:
        #     twist_msg.linear.x = 0.4
        # elif command >= 71 and command <= 150:
        #     twist_msg.linear.x = -0.4 
        # else:
            print("Invalid command")

        self.twist_publisher.publish(twist_msg)

        if twist_msg.angular.z != 0:
#            rotation_duration = abs(command - 75) / 50  # Adjust duration based on command
            rotation_duration = 1  # Adjust duration based on command
            rotation_timer = threading.Timer(rotation_duration, self.stop_movement)
            rotation_timer.start()
        elif twist_msg.linear.x != 0:
            #movement_duration = abs(command - 125) / 50  # Adjust duration based on command
            movement_duration = 1 # Adjust duration based on command
            movement_timer = threading.Timer(movement_duration, self.stop_movement)
            movement_timer.start()

    def stop_movement(self):
        twist_msg = Twist()
        self.twist_publisher.publish(twist_msg)
        print("stopped")


    def print_position_and_motion(self):
        if self.odometry_data is not None:
            position = self.odometry_data.pose.pose.position
            motion = self.odometry_data.twist.twist
            orientation = self.odometry_data.pose.pose.orientation
            
            # Convert orientation to Euler angles (roll, pitch, yaw)
            quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
            roll, pitch, yaw = tft.euler_from_quaternion(quaternion)

            print("Position data: [x={}, y={}, z={}]\nMotion data: [linear velocity={}, angular velocity={}, yaw={}]".format(
                position.x, position.y, position.z, motion.linear.x, motion.angular.z, yaw))
        else:
            print("No odometry data available yet")







    ###############################################################################     ROS CALLBACKS
        
    '''
    BehaviorTreeLog callback
    '''
    def bt_callback(self, msg):
        # print(msg)
        for event in msg.event_log:
            if event.node_name == 'NavigateRecovery' and event.current_status == 'IDLE':
                self.btReady = True

    '''
    Map subscription callback
    '''
    def map_callback(self, msg):
        self.map = OccupancyGrid2d(msg)
        if self.map is not None:
            self.updatedMap = True
    
    '''
    LiDAR subscription callback
    '''
    def lidar_callback(self, msg):
        
        self.lidar = msg
    
    '''
    Odometry subscription callback
    '''
    # def odom_callback(self, msg):
    #     self.updatedPose = True
    #     self.currentPose = msg.pose.pose
    #     self.initial_pose_received = True

    def odom_callback(self, msg):
        self.odometry_data = msg
        if self.odometry_data is not None:

            position = self.odometry_data.pose.pose.position
            motion = self.odometry_data.twist.twist
            odom_data = {
                "position": {"x": position.x, "y": position.y, "z": position.z},
                "motion": {"linear_velocity": motion.linear.x, "angular_velocity": motion.angular.z}
            }
            self.publish_odometry_data(odom_data)
            #position = self.odometry_data.pose.pose.position
            #motion = self.odometry_data.twist.twist

            # self.last_pos_x = position.x
            # self.last_pos_y = position.y
            # self.last_pos_z = position.z
            # self.last_mot_x = motion.linear.x
            # self.last_mot_z = motion.angular.z

    # def publish_odometry_data(self, odom_data):
    #     self.mqttc.publish("un44889195", json.dumps(odom_data))



    
    def publishInitialPose(self):
        '''
        Publishes the initial pose self.init_pose
        '''
        self.initial_pose_pub.publish(self.init_pose)
    
    def setInitialPose(self, pose):
        '''
        Sets the initial pose
        '''
        self.init_pose = PoseWithCovarianceStamped()
        self.init_pose.pose.pose.position.x = pose[0]
        self.init_pose.pose.pose.position.y = pose[1]
        self.init_pose.header.frame_id = 'map'
        self.currentPose = self.init_pose.pose.pose
        self.publishInitialPose()
        run_with_timeout(self, timeout_sec=5.0)

    def info_msg(self, msg: str):
        self.get_logger().info(msg)

    def warn_msg(self, msg: str):
        self.get_logger().warn(msg)

    def error_msg(self, msg: str):
        self.get_logger().error(msg) 


    ###############################################################################     MQTT

    def on_subscribe(self, client, userdata, mid, reason_code_list, properties):
        if reason_code_list[0].is_failure:
            print(f"Broker rejected your subscription: {reason_code_list[0]}")
        else:
            print(f"Broker granted the following QoS: {reason_code_list[0].value}")

    def on_unsubscribe(self, client, userdata, mid, reason_code_list, properties):
        if len(reason_code_list) == 0 or not reason_code_list[0].is_failure:
            print("Unsubscribe succeeded.")
        else:
            print(f"Broker replied with failure: {reason_code_list[0]}")
        client.disconnect()

    def on_message(self, client, userdata, message):
        inval = message.payload.decode()
        input_array = self.parse_mqtt_string(inval)
        #print("got message ", input_array)
        print("got message ", input_array[0],input_array[1],input_array[2],input_array[3],input_array[4])
        self.last_rec_command = input_array

    

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            print(f"Failed to connect: {reason_code}. loop_forever() will retry connection")
        else:
            client.subscribe("un44779195")

    def publish_odometry_data(self, odom_data):
        self.mqttc.publish("46591300_General", json.dumps(odom_data))


    def parse_mqtt_string(self,mqtt_string):
        # Remove the square brackets and split the string by commas
        split_string = mqtt_string.strip("[]").split(",")

        # Convert the string elements to integers
        parsed_array = [int(element.strip()) for element in split_string]

        return parsed_array


    def mqtt_process(self):
        self.mqttc.loop_forever()









def main(args=None):
    rclpy.init(args=args)
    gestcont = Gestcont()
    rclpy.spin(gestcont)
    gestcont.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
