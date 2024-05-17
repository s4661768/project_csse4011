import rclpy
from geometry_msgs.msg import Twist
from pynput import keyboard

# Define key mappings for movement
KEY_MAPPING = {
    keyboard.Key.up: (0.1, 0.0),     # Forward
    keyboard.Key.down: (-0.1, 0.0),   # Backward
    keyboard.Key.left: (0.0, 0.1),    # Rotate left
    keyboard.Key.right: (0.0, -0.1)   # Rotate right
}

# Initialize the Twist message publisher
def init_publisher(node):
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    return publisher

# Callback function for keyboard events
def on_press(key):
    if key in KEY_MAPPING:
        linear_vel, angular_vel = KEY_MAPPING[key]
        publish_twist(linear_vel, angular_vel)

def on_release(key):
    if key == keyboard.Key.esc:
        # Stop the TurtleBot3 if the Escape key is pressed
        publish_twist(0.0, 0.0)
        return False

# Publish Twist message with given linear and angular velocities
def publish_twist(linear_vel, angular_vel):
    twist_msg = Twist()
    twist_msg.linear.x = linear_vel
    twist_msg.angular.z = angular_vel
    publisher.publish(twist_msg)

def main():
    rclpy.init()
    node = rclpy.create_node('keyboard_controller')
    global publisher
    publisher = init_publisher(node)

    # Create listener for keyboard events
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
