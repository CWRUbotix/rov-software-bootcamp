from time import sleep

import cv2

from bootcamp_harness.rclpy.node import Node
from bootcamp_harness.rclpy.qos import QoSPresetProfiles
from bootcamp_harness.sensor_msgs.msg import Image
from bootcamp_harness import rclpy

if __name__ == '__main__':
    video_capture = cv2.VideoCapture('video.mkv')

    rclpy.init()

    node = Node('front_cam')

    publisher = node.create_publisher(Image, 'front_cam/image_raw',
                                      QoSPresetProfiles.DEFAULT.value)

    while video_capture.isOpened():
        ret, cv2_frame = video_capture.read()
        img_msg = Image(cv2_frame)
        publisher.publish(img_msg)
        sleep(1 / 30)
    
    video_capture.release()
