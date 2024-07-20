from bootcamp_harness.rclpy.node import Node
from bootcamp_harness.rclpy.qos import QoSPresetProfiles
from bootcamp_harness.rov_msgs.msg import PixhawkInstruction
from bootcamp_harness import rclpy


def subscription_callback(message: PixhawkInstruction):
    print(f'Received: "{message}"')


if __name__ == '__main__':
    rclpy.init()

    node = Node('pixhawk_subscriber')

    node.create_subscription(PixhawkInstruction, 'pixhawk_control',
                             subscription_callback,  # TODO: why doesn't Mypy like this?
                             QoSPresetProfiles.DEFAULT.value)
    
    rclpy.spin(node)
