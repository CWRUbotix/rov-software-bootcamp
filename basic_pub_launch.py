from time import sleep

from bootcamp_harness.rclpy.node import Node
from bootcamp_harness.rclpy.qos import QoSPresetProfiles
from bootcamp_harness import rclpy

def main():
    rclpy.init()

    node = Node('my_publisher_node')

    publisher = node.create_publisher(str, 'my_topic',
                                      QoSPresetProfiles.DEFAULT.value)

    i = 0
    while True:
        msg = f'Message {i}'
        print(f'Publishing "{msg}"')
        publisher.publish(msg)
        i += 1
        sleep(1)

if __name__ == '__main__':
    main()
