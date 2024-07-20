from typing import TypeVar, Callable
import re

from bootcamp_harness.rclpy.qos import QoSProfile
from bootcamp_harness.rclpy.subscription import Subscription
from bootcamp_harness.rclpy.publisher import Publisher

SNAKE_ILLEGAL_CHARS = re.compile(r'[^a-z0-9_]')

MsgType = TypeVar('MsgType')


class Node:
    """
    A Node in the ROS graph.

    A Node is the primary entrypoint in a ROS system for communication.
    It can be used to create ROS entities such as publishers, subscribers, services, etc.
    """

    def __init__(self, node_name: str):
        """
        Create a Node.

        :param node_name: A name to give to this node. Should be in snake_case.
        """
        if not Node._is_snake(node_name):
            raise ValueError('Node name not in snake case!')

        self.subscriptions: list[Subscription] = []
        self.publishers: list[Publisher] = []

    def create_publisher(
        self,
        msg_type: MsgType,
        topic: str,
        qos_profile: QoSProfile,
    ) -> Publisher:
        """
        Create a new publisher.

        :param msg_type: The type of ROS messages the publisher will publish.
        :param topic: The name of the topic the publisher will publish to.
        :param qos_profile: A QoSProfile or a history depth to apply to the publisher.
            In the case that a history depth is provided, the QoS history is set to
            KEEP_LAST, the QoS history depth is set to the value
            of the parameter, and all other QoS settings are set to their default values.
            **CWRUbotix tip: set this to rclpy.qos.QoSPresetProfiles.DEFAULT.value**
        :return: The new publisher.
        """

        publisher = Publisher(msg_type, topic, qos_profile)
        self.publishers.append(publisher)
        return publisher

    def create_subscription(
        self,
        msg_type: MsgType,
        topic: str,
        callback: Callable[[MsgType], None],
        qos_profile: QoSProfile,
    ) -> Subscription:
        """
        Create a new subscription.

        :param msg_type: The type of ROS messages the subscription will subscribe to.
        :param topic: The name of the topic the subscription will subscribe to.
        :param callback: A user-defined callback function that is called when a message is
            received by the subscription.
        :param qos_profile: A QoSProfile or a history depth to apply to the subscription.
            In the case that a history depth is provided, the QoS history is set to
            KEEP_LAST, the QoS history depth is set to the value
            of the parameter, and all other QoS settings are set to their default values.
        """

        subscription = Subscription(msg_type, topic, callback, qos_profile)
        self.subscriptions.append(subscription)

        return subscription

    @staticmethod
    def _is_snake(string: str):
        return not bool(SNAKE_ILLEGAL_CHARS.search(string))
