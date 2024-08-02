from bootcamp_harness.rclpy.node import Node
from bootcamp_harness.rclpy.qos import QoSPresetProfiles
from bootcamp_harness import rclpy


def subscription_callback(message: str) -> None:
    print(f'Received: "{message}"')


if __name__ == '__main__':
    rclpy.init()

    node = Node('my_subscription_node')

    node.create_subscription(str, 'my_topic', subscription_callback,  # TODO: why doesn't Mypy like this?
                             QoSPresetProfiles.DEFAULT.value)
    
    rclpy.spin(node)
