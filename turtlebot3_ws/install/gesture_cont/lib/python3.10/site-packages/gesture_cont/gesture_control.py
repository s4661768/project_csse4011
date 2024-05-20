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
import numpy as np

import time
import threading
import json


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


        #POSE DATA
        self.pose_data = None
        self.cov = 3

        #ODOMETRY STUFF
        self.odometry_data = None
        self.current_odom_x = 0             #current actual location
        self.current_odom_y = 0
        self.x_offset = 0               #current offset
        self.y_offset = 0

        self.initial_offset_flag = 0
        #Gesture control
        self.last_rec_command = 1





        # Set the log level to INFO
        self.get_logger().set_level(LoggingSeverity.INFO)

        #subs

        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'pose',
            self.pose_callback,
            10
        )
      
        # prevent unused variable warning
        self.odom_subscription
        self.pose_subscription

        # Set up publisher
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)


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

        
        #set up the mqtt
        self.mqtt_thread = threading.Thread(target=self.mqtt_process,daemon=True).start()
        
        # Start controller
        self.gesture_control_protocol()




    ###############################################################################     GESTURE CONTROLLER

    def gesture_control_protocol(self):
        while True:
            print("last recieved command raw", self.last_rec_command)
  
            self.control_move(self.last_rec_command)
            
            self.print_position_and_motion()

            rclpy.spin_once(self, timeout_sec=1.0)  # Process messages continuously
            time.sleep(2)


    def control_move(self, command):
        twist_msg = Twist()

        if command == 0:
            twist_msg.linear.x = 0.0          #null command, stop
            twist_msg.angular.z = 0.0
        elif command == 1:
            twist_msg.linear.x = 0.1        #go forward
        elif command == 2:
            twist_msg.linear.x = -0.1       #go backward
        elif command == 3:
            twist_msg.angular.z = 0.5 
        elif command == 4:
            twist_msg.angular.z = -0.5  # Turn right
        elif command == 5:
            self.pos_offset()
        else:

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


    def print_position_and_motion(self):
        if self.odometry_data is not None:
            position = self.odometry_data.pose.pose.position
            motion = self.odometry_data.twist.twist
            orientation = self.odometry_data.pose.pose.orientation
            
            # Convert orientation to Euler angles (roll, pitch, yaw)
            quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
            roll, pitch, yaw = tft.euler_from_quaternion(quaternion)

           # print("Position data: [x={}, y={}, z={}]\nMotion data: [linear velocity={}, angular velocity={}, yaw={}]".format(
            #position.x, position.y, position.z, motion.linear.x, motion.angular.z, yaw
        #else:
           # print("No odometry data available yet")



    def pos_offset(self):
        self.x_offset = self.current_odom_x + 1
        self.y_offset = self.current_odom_y + 1


    '''
    Odometry subscription callback
    '''

    def pose_callback(self,msg):
        self.pose_data = msg
        if self.pose_data is not None:
            print("in")
            position = self.pose_data.pose.pose.position
            print("POSE POS", position)
            updatexy, self.cov = self.kalmanfunc(self.current_odom_x, self.current_odom_y, self.cov, position.x, position.y)

            print("updatexy",updatexy)
            print("cov", self.cov)
            self.current_odom_x = updatexy[0]
            self.current_odom_y = updatexy[1]

    
    def odom_callback(self, msg):
        self.odometry_data = msg
        if self.odometry_data is not None:


            position = self.odometry_data.pose.pose.position
            motion = self.odometry_data.twist.twist
            print("ODOM POS", position)
            orientation = self.odometry_data.pose.pose.orientation
            # Convert orientation to Euler angles (roll, pitch, yaw)
            quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
            roll, pitch, yaw = tft.euler_from_quaternion(quaternion)


            self.current_odom_x = position.x
            self.current_odom_y = position.y

            adj_x = (self.x_offset - position.x )
            adj_y = (self.y_offset - position.y)

            odom_send = {
                "position": {"x": adj_x, "y": adj_y, "Yaw":yaw},
                "motion": {"linear_velocity": motion.linear.x, "angular_velocity": motion.angular.z}

            }
            self.publish_odometry_data(odom_send)
   

    def info_msg(self, msg: str):
        self.get_logger().info(msg)

    def warn_msg(self, msg: str):
        self.get_logger().warn(msg)

    def error_msg(self, msg: str):
        self.get_logger().error(msg)


    ###############################################################################     KALMAN
    def kalmanfunc(self,odomx, odomy, covscalar, posx,posy):
        #inits

        Q_k = np.eye(4) * 1
        dt = 0.2
        A = np.array([(1, 0, dt, 0), (0, 1, 0, dt), (0, 0, 1, 0), (0, 0, 0, 1)])
        H = np.array([(1, 0, 0, 0), (0, 1, 0, 0)])
        R = np.eye(len(H))*0.4
        cov = covscalar * np.eye(4) 
        
        # Make prediction  
        x_hat_est = np.array( [odomx, odomy, 0, 0] )
        cov_est = np.dot(A,np.dot(cov,np.transpose(A))) + Q_k
        obs = np.array([posx,posy])

        # Update estimate
        error_x = obs - np.dot(H,x_hat_est)
        error_cov = np.dot(H,np.dot(cov_est,np.transpose(H))) + R
        K = np.dot(np.dot(cov_est,np.transpose(H)),np.linalg.inv(error_cov))
        x_hat = x_hat_est + np.dot(K,error_x)
        cov = np.dot((np.eye(4) - np.dot(K,H)),cov_est)

        postup = (x_hat[0], x_hat[1])
        covscalar = cov[0,0]

        return postup, covscalar


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
      #  print("got message ", message)
        inval = message.payload.decode()
      #  print("got decode ", inval)
       # print("decode type", type(inval))
       # print("decode shape", len(inval))
        input_array = int(inval.strip('"'))
        #input_array = self.parse_mqtt_string(int(inval)) 
       # print("got message array ", input_array)
        #print("got message ", input_array[0],input_array[1],input_array[2],input_array[3],input_array[4])
        if (input_array != None):
        
            self.last_rec_command = input_array
            print("last rec commnad ", self.last_rec_command)

    

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            print(f"Failed to connect: {reason_code}. loop_forever() will retry connection")
        else:
            client.subscribe("un44779195")

    def publish_odometry_data(self, odom_data):
        self.mqttc.publish("un46591300", json.dumps(odom_data))


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
