from time import sleep

from bootcamp_harness.rclpy.node import Node
from bootcamp_harness.rclpy.qos import QoSPresetProfiles
from bootcamp_harness.rov_msgs.msg import PixhawkInstruction
from bootcamp_harness import rclpy

if __name__ == '__main__':
    rclpy.init()

    node = Node('pixhawk_publisher')

    publisher = node.create_publisher(PixhawkInstruction, 'pixhawk_control',
                                      QoSPresetProfiles.DEFAULT.value)

    while True:
        instruction = PixhawkInstruction(forward=0.5, lateral=1.0, author=PixhawkInstruction.MANUAL_CONTROL)
        publisher.publish(instruction)
        sleep(1)
