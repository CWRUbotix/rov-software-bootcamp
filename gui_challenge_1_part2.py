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

# enum for movement
class MovementType(Enum):
    Forward = 1  # Forward/backward
    Vertical = 2  # Up/down
    Lateral = 3  # Left/right
    Pitch = 4  # Tilting up/down
    Yaw = 5  # Turning left/right
    Stop = 6 # stop - added for challege
    
# main class for button screen 
class ButtonPanel(QWidget):
    # create signal
    handle_frame_signal = pyqtSignal(Image)
    def __init__(self) -> None:
        super().__init__()
         
        # signal slot connection
        self.handle_frame_signal.connect(self.handle_frame)
        
        # create ros nodes
        node = Node('pixhawk_publisher')
        subscriber_node = Node('camera_gui_subscriber')
        
        # initializes a subscription
        subscriber_node.create_subscription(
        Image,
        'front_cam/image_raw',
        lambda message: self.handle_frame_signal.emit(message),
        QoSPresetProfiles.DEFAULT.value
    )
        # node is in seperate thred
        executor = SingleThreadedExecutor()
        executor.add_node(subscriber_node)
        Thread(target=executor.spin, daemon=True).start()
        
        # initialize a publisher 
        self.pixhawk_publisher = node.create_publisher(
            PixhawkInstruction,
            'pixhawk_control',
            QoSPresetProfiles.DEFAULT.value
        )
        
        # create layout
        layout = QVBoxLayout()
        self.setLayout(layout)
        
        # create a layout for the video
        image_layout = QHBoxLayout()
        layout.addLayout(image_layout)
        
        # create a label for the video
        self.video_frame_label = QLabel()
        image_layout.addWidget(self.video_frame_label)
        
         # create top layout
        top_layout = QHBoxLayout()
        layout.addLayout(top_layout)
        
        # top row buttons
        left = QPushButton('left')
        top_layout.addWidget(left)
        forward = QPushButton('forward')
        top_layout.addWidget(forward)
        up = QPushButton('up')
        top_layout.addWidget(up)
        tilt_up = QPushButton('tilt up')
        top_layout.addWidget(tilt_up)
        turn_left = QPushButton('turn left')
        top_layout.addWidget(turn_left)
        
        #added for challenge
        stop = QPushButton('stop')
        top_layout.addWidget(stop)
     
        # create bottom layout
        bottom_layout = QHBoxLayout()
        layout.addLayout(bottom_layout)
        
        # bottom row buttons
        right = QPushButton('right')
        bottom_layout.addWidget(right)
        backward = QPushButton('back')
        bottom_layout.addWidget(backward)
        down = QPushButton('down')
        bottom_layout.addWidget(down)
        tilt_down = QPushButton('tilt down')
        bottom_layout.addWidget(tilt_down)
        turn_right = QPushButton('turn right')
        bottom_layout.addWidget(turn_right)
        
        # connect buttons to methods that make to buttons do things
        forward.clicked.connect(
            lambda: self.on_button_press(MovementType.Forward, True))
        backward.clicked.connect(
            lambda: self.on_button_press(MovementType.Forward, False))
        up.clicked.connect(
            lambda: self.on_button_press(MovementType.Vertical, True))
        down.clicked.connect(
            lambda: self.on_button_press(MovementType.Vertical, False))
        left.clicked.connect(
            lambda: self.on_button_press(MovementType.Lateral, True))
        right.clicked.connect(
            lambda: self.on_button_press(MovementType.Lateral, False))
        tilt_up.clicked.connect(
            lambda: self.on_button_press(MovementType.Pitch, True))
        tilt_down.clicked.connect(
            lambda: self.on_button_press(MovementType.Pitch, False))
        turn_left.clicked.connect(
            lambda: self.on_button_press(MovementType.Yaw, True))
        turn_right.clicked.connect(
            lambda: self.on_button_press(MovementType.Yaw, False))
        
        # added for challenge
        stop.clicked.connect(
            lambda: self.on_button_press(MovementType.Stop, False)    
        )
        

    
    # method for what to do when a button is pressed
    def on_button_press(self, movement_type: MovementType, direction: bool):
        direction_str = 'positively' if direction else 'negatively'
        print(f'Moving: {movement_type.name} {direction_str}')
    
        
        # create a pixhawk instruction
        value = 0.5 if direction else -0.5
        
        
        instruction = PixhawkInstruction(
                          x=(value if movement_type == MovementType.Forward else 0),
                          z=(value if movement_type == MovementType.Vertical else 0),
                          y=(value if movement_type == MovementType.Lateral else 0),
                          yaw =(value if movement_type == MovementType.Yaw else 0),
                          pitch =(value if movement_type == MovementType.Pitch else 0),
                          author=PixhawkInstruction.MANUAL_CONTROL
                      )
        # publish the pixhawk instruction
        self.pixhawk_publisher.publish(instruction)
    
    
    # Challenge 1 part 2 mostly below
        
    # what is done when gui recieves the image
    @pyqtSlot(Image)
    def handle_frame(self, ros_image: Image) -> None:
        print('Got an image!')
        # create cv bridge
        self.cv_bridge = CvBridge()
        # convert to cv
        self.cv_image = self.cv_bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
        # convert to qt 
        self.qt_image: QImage = self.convert_cv_qt(self.cv_image, 890, 682)
        # add it to the gui
        self.video_frame_label.setPixmap(QPixmap.fromImage(self.qt_image))
        
        
    # converts cv to qt (given in tutorial)
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
    
    # enables ros
    rclpy.init()
    
    # initializes app
    app = QApplication([])

    # creates a window to show our gui
    window = ButtonPanel()
    window.show()

    # executes app
    app.exec()

# python main function stuff
if __name__ == '__main__':
    main()