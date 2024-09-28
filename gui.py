from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton
from enum import Enum
from bootcamp_harness.rclpy.node import Node
from bootcamp_harness.rclpy.qos import QoSPresetProfiles
from bootcamp_harness.rov_msgs.msg import PixhawkInstruction
from bootcamp_harness import rclpy
from threading import Thread
from numpy.typing import NDArray

from PyQt6.QtCore import Qt, pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import QLabel
from PyQt6.QtGui import QImage, QPixmap

from bootcamp_harness.sensor_msgs.msg import Image
from bootcamp_harness.cv_bridge import CvBridge
from bootcamp_harness.rclpy.executors import SingleThreadedExecutor

import cv2
import numpy as np

class MovementType(Enum):
    Stop = 0 # Stop the robot
    Forward = 1  # Forward/backward
    Vertical = 2  # Up/down
    Lateral = 3  # Left/right
    Pitch = 4  # Tilting up/down
    Yaw = 5  # Turning left/right

class ButtonPanel(QWidget):
    def __init__(self) -> None:
        super().__init__()

        node = Node('pixhawk_publisher')
        self.pixhawk_publisher = node.create_publisher(
            PixhawkInstruction,
            'pixhawk_control',
            QoSPresetProfiles.DEFAULT.value
        )
        
        layout = QVBoxLayout()
        self.setLayout(layout)

        top_layout = QHBoxLayout()
        layout.addLayout(top_layout)
         
        bottom_layout = QHBoxLayout()
        layout.addLayout(bottom_layout)

        top_button_types = [[MovementType.Vertical, True], [MovementType.Yaw, True], [MovementType.Forward, True], [MovementType.Yaw, False], [MovementType.Pitch, True]]
        bottom_button_types = [[MovementType.Vertical, False], [MovementType.Lateral, True], [MovementType.Forward, False], [MovementType.Lateral, False], [MovementType.Pitch, False], [MovementType.Stop, False]]

        for type in top_button_types:
            direction = "+" if type[1] else "-"
            button_name = direction + type[0].name
            button = QPushButton(button_name)
            top_layout.addWidget(button)
            button.clicked.connect(lambda: self.on_button_press(type[0], type[1]))

        for type in bottom_button_types:
            direction = "+" if type[1] else "-"
            button_name = direction + type[0].name
            button = QPushButton(button_name)
            bottom_layout.addWidget(button)
            button.clicked.connect(lambda: self.on_button_press(type[0], type[1]))


        # upward = QPushButton('up')
        # top_layout.addWidget(upward)
        # upward.clicked.connect(lambda: self.on_button_press(MovementType.Vertical, True))

        # turn_left = QPushButton('turn left')
        # top_layout.addWidget(turn_left)
        # turn_left.clicked.connect(lambda: self.on_button_press(MovementType.Yaw, True))

        # forward = QPushButton('forward')
        # top_layout.addWidget(forward)
        # forward.clicked.connect(lambda: self.on_button_press(MovementType.Forward, True))

        # turn_right = QPushButton('turn right')
        # top_layout.addWidget(turn_right)
        # turn_right.clicked.connect(lambda: self.on_button_press(MovementType.Yaw, False))

        # tilt_up = QPushButton('tilt up')
        # top_layout.addWidget(tilt_up)
        # tilt_up.clicked.connect(lambda: self.on_button_press(MovementType.Pitch, True))

        # downward = QPushButton('down')
        # bottom_layout.addWidget(downward)
        # downward.clicked.connect(lambda: self.on_button_press(MovementType.Vertical, False))

        # left = QPushButton('left')
        # bottom_layout.addWidget(left)
        # left.clicked.connect(lambda: self.on_button_press(MovementType.Lateral, True))

        # backward = QPushButton('backward')
        # bottom_layout.addWidget(backward)
        # backward.clicked.connect(lambda: self.on_button_press(MovementType.Forward, False))

        # right = QPushButton('right')
        # bottom_layout.addWidget(right)
        # right.clicked.connect(lambda: self.on_button_press(MovementType.Lateral, False))

        # tilt_down = QPushButton('tilt down')
        # bottom_layout.addWidget(tilt_down)
        # tilt_down.clicked.connect(lambda: self.on_button_press(MovementType.Pitch, False))

        # stop = QPushButton('stop')
        # bottom_layout.addWidget(stop)
        # stop.clicked.connect(lambda: self.on_button_press(MovementType.Stop, False))

        video_layout = QHBoxLayout()
        layout.addLayout(video_layout)

        front_video = VideoPanel('front_cam/image_raw')
        layout.addWidget(front_video)

        down_video = VideoPanel('down_cam/image_raw')
        layout.addWidget(down_video)

    def on_button_press(self, movement_type: MovementType, direction:bool):
        direction_str = 'positively' if direction else 'negatively'

        value = 0.5 if direction else -0.5
        value = 0 if movement_type == MovementType.Stop else value

        instruction = PixhawkInstruction(
            forward= (value if movement_type == MovementType.Forward else 0),
            vertical= (value if movement_type == MovementType.Vertical else 0),
            lateral= (value if movement_type == MovementType.Lateral else 0),
            pitch = (value if movement_type == MovementType.Pitch else 0),
            yaw = (value if movement_type == MovementType.Yaw else 0),
            author=PixhawkInstruction.MANUAL_CONTROL
        )
        self.pixhawk_publisher.publish(instruction)
        print(f'Moving: {movement_type.name} {direction_str}')

class VideoPanel(QWidget):

    handle_frame_signal = pyqtSignal(Image)

    def __init__(self, camera_subscription: str) -> None:
        super().__init__()

        node = Node('pixhawk_publisher')
        node.create_subscription(
            Image,
            camera_subscription,
            lambda message: self.handle_frame_signal.emit(message),
            QoSPresetProfiles.DEFAULT.value
        )

        self.handle_frame_signal.connect(self.handle_frame)

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        Thread(target=executor.spin, daemon=True).start()

        layout = QHBoxLayout()
        self.setLayout(layout)
        self.video_frame_label = QLabel()
        layout.addWidget(self.video_frame_label)
        self.video_frame_label_annotated = QLabel()
        layout.addWidget(self.video_frame_label_annotated)

    @pyqtSlot(Image)
    def handle_frame(self, ros_image: Image) -> None:
            self.cv_bridge = CvBridge()
            cv_image: NDArray = self.cv_bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
            if not cv_image is None:
                # This is the code for the original frame
                qt_image: QImage = self.convert_cv_qt(cv_image, 445, 341)
                self.video_frame_label.setPixmap(QPixmap.fromImage(qt_image))

                # This is the code to add the border to the frame
                hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
                
                bottom_red_lower = np.array([0, 50, 50])
                bottom_red_upper = np.array([10, 255, 255])
                top_red_lower = np.array([170, 50, 50])
                top_red_upper = np.array([180, 255, 255])
                
                bottom_mask = cv2.inRange( hsv_frame, bottom_red_lower, bottom_red_upper )
                top_mask = cv2.inRange(hsv_frame, top_red_lower, top_red_upper)
                
                mask = bottom_mask + top_mask

                contours, _ = cv2.findContours(
                    mask.copy(),
                    cv2.RETR_TREE,
                    cv2.CHAIN_APPROX_SIMPLE
                )

                red_contour = contours[0]
                if len(contours) > 0:
                    biggest_contour = max(contours, key=cv2.contourArea)
                    red_contour = biggest_contour

                x, y, w, h = cv2.boundingRect(red_contour)

                cv2.rectangle(mask, (x, y), (x + w, y + h), (100, 255, 255), 2)

                qt_image: QImage = self.convert_cv_qt(mask, 445, 341)
                self.video_frame_label_annotated.setPixmap(QPixmap.fromImage(qt_image))
            else:
                self.video_frame_label.setText("The video has ended")
                
    
    def convert_cv_qt(self, cv_img: NDArray, width: int = 0, height: int = 0) -> QImage:
        """Convert from an opencv image to QPixmap."""
        # Color image
        if len(cv_img.shape) == 3:
            h, w, ch = cv_img.shape
            bytes_per_line = ch * w

            img_format = QImage.Format.Format_BGR888

        # Grayscale image
        elif len(cv_img.shape) == 2:
            h, w = cv_img.shape
            bytes_per_line = w

            img_format = QImage.Format.Format_Grayscale8

        else:
            raise ValueError('Somehow not color or grayscale image.')

        qt_image = QImage(cv_img.data, w, h, bytes_per_line, img_format)
        qt_image = qt_image.scaled(width, height, Qt.AspectRatioMode.KeepAspectRatio)

        return qt_image
    

def main():
    rclpy.init()
    app = QApplication([])

    window = ButtonPanel()
    window.show()

    app.exec()

if __name__ == '__main__':
    main()