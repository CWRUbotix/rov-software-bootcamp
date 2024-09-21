from .sensor_msgs.msg import Image, MatLike


class CvBridge:
    def __init__(self):
        pass

    def imgmsg_to_cv2(self, frame: Image,
                      desired_encoding: str = 'passthrough') -> MatLike:
        """
        Convert a sensor_msgs::Image message to an OpenCV :cpp:type:`cv::Mat`.

        :param img_msg:   A :cpp:type:`sensor_msgs::Image` message
        :param desired_encoding:  The encoding of the image data, one of the following strings:

           * ``"passthrough"``
           * one of the standard strings in sensor_msgs/image_encodings.h

        :rtype: :cpp:type:`cv::Mat`
        :raises CvBridgeError: when conversion is not possible.

        If desired_encoding is ``"passthrough"``, then the returned image has the same format
        as img_msg. Otherwise desired_encoding must be one of the standard image encodings

        This function returns an OpenCV :cpp:type:`cv::Mat` message on success,
        or raises :exc:`cv_bridge.CvBridgeError` on failure.

        If the image only has one channel, the shape has size 2 (width and height)
        """

        if desired_encoding != 'passthrough':
            raise ValueError('CWRUbotix tip: Your imgmsg_to_cv2'
                             'desired_encoding should be "passthrough"')

        return frame.secret_internal_image
