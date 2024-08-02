from typing import TypeVar, Callable, Generic
import pickle

import zmq

from .qos import QoSProfile, QoSPresetProfiles
from .broker import SUB_SOCKET_URL

MsgType = TypeVar('MsgType')


class Subscription(Generic[MsgType]):
    def __init__(
         self,
         msg_type: type[MsgType],
         topic: str,
         callback: Callable[[MsgType], None],
         qos_profile: QoSProfile,
    ) -> None:
        """
        Create a container for a ROS subscription.

        .. warning:: Users should not create a subscription with this constructor, instead they
           should call :meth:`.Node.create_subscription`.

        :param msg_type: The type of ROS messages the subscription will subscribe to.
        :param topic: The name of the topic the subscription will subscribe to.
        :param callback: A user-defined callback function that is called when a message is
            received by the subscription.
        :param qos_profile: The quality of service profile to apply to the subscription.
        """

        if qos_profile != QoSPresetProfiles.DEFAULT.value:
            raise ValueError('CWRUbotix tip: your qos_profile should be'
                             'QoSPresetProfiles.DEFAULT.value')

        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback
        self.qos_profile = qos_profile

        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.connect(SUB_SOCKET_URL)
        self.socket.subscribe(topic)

        print(f'Subscriber connected to topic {topic}')

    def secret_internal_poll(self) -> None:
        # Format: [topic, message], where both indices are "bytes" types
        multipart_packet = self.socket.recv_multipart()

        if len(multipart_packet) != 2:
            print(f'Subscription on {self.topic} failed to receive message '
                  'because the multipart data had the wrong length')

        message = pickle.loads(multipart_packet[1])

        if not isinstance(message, self.msg_type):  # TODO: why doesn't Mypy like this?
            print(f'Subscription on {self.topic} failed to process message '
                  'payload because it was not the correct type')
            return

        self.callback(message)
