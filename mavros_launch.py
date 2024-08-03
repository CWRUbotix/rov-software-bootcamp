from bootcamp_harness.rclpy.node import Node
from bootcamp_harness.rclpy.qos import QoSPresetProfiles
from bootcamp_harness.rov_msgs.msg import PixhawkInstruction
from bootcamp_harness import rclpy

def subscription_callback(message: PixhawkInstruction) -> None:
    print(f'[Pixhawk] Received: "{message}"')

def main():
    rclpy.init()

    node = Node('mavros_node')

    node.create_subscription(PixhawkInstruction, 'pixhawk_control',
                             subscription_callback,
                             QoSPresetProfiles.DEFAULT.value)
    
    rclpy.spin(node)

if __name__ == '__main__':
    main()
