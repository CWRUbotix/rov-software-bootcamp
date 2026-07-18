# Not too many resources that show you how to do this
# used https://zetcode.com/pyqt/qshortcut/ , converted from PyQt5
# Overall pretty easy, hardest part was finding the message

from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton
from PyQt6.QtGui import QShortcut, QKeySequence
from enum import Enum
from PyQt6.QtCore import Qt

from bootcamp_harness.rclpy.node import Node
from bootcamp_harness.rclpy.qos import QoSPresetProfiles
from bootcamp_harness.rov_msgs.msg import PixhawkInstruction
from bootcamp_harness import rclpy

# enum for movement
class MovementType(Enum):
    Forward = 1  # Forward/backward
    Vertical = 2  # Up/down
    Lateral = 3  # Left/right
    Pitch = 4  # Tilting up/down
    Yaw = 5  # Turning left/right
    Stop = 6 # stop
    
# main class for button screen 
class ButtonPanel(QWidget):
    def __init__(self) -> None:
        super().__init__()
        
        # create a ros node
        node = Node('pixhawk_publisher')
        
        # initialize a publisher 
        self.pixhawk_publisher = node.create_publisher(
            PixhawkInstruction,
            'pixhawk_control',
            QoSPresetProfiles.DEFAULT.value
        )
        
        # create layout
        layout = QVBoxLayout()
        self.setLayout(layout)
        
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
        last_key = QPushButton('none')
        bottom_layout.addWidget(last_key)
        
        # challenge - works with keyboard keys
        self.upSc = QShortcut(QKeySequence(Qt.Key.Key_Up), self)
        self.downSc = QShortcut(QKeySequence(Qt.Key.Key_Down), self)
        self.leftSc = QShortcut(QKeySequence(Qt.Key.Key_Left), self)
        self.rightSc = QShortcut(QKeySequence(Qt.Key.Key_Right), self)
        self.forwardSc = QShortcut(QKeySequence("w"), self)
        self.backSc = QShortcut(QKeySequence("s"), self)
        self.turn_leftSc = QShortcut(QKeySequence("a"), self)
        self.turn_rightSc = QShortcut(QKeySequence("d"), self)
        self.tilt_upSc = QShortcut(QKeySequence("q"), self)
        self.tilt_downSc = QShortcut(QKeySequence("e"), self)
        self.stopSc = QShortcut(QKeySequence("z"), self)
        
        self.upSc.activated.connect( lambda: self.on_button_press(MovementType.Vertical, True))
        self.downSc.activated.connect( lambda: self.on_button_press(MovementType.Vertical, False))
        self.forwardSc.activated.connect( lambda: self.on_button_press(MovementType.Forward, True))
        self.backSc.activated.connect( lambda: self.on_button_press(MovementType.Forward, False))
        self.leftSc.activated.connect( lambda: self.on_button_press(MovementType.Lateral, True))
        self.rightSc.activated.connect( lambda: self.on_button_press(MovementType.Lateral, False))
        self.tilt_upSc.activated.connect( lambda: self.on_button_press(MovementType.Pitch, True))
        self.tilt_downSc.activated.connect( lambda: self.on_button_press(MovementType.Pitch, False))
        self.turn_leftSc.activated.connect( lambda: self.on_button_press(MovementType.Yaw, True))
        self.turn_rightSc.activated.connect( lambda: self.on_button_press(MovementType.Yaw, False))
        self.stopSc.activated.connect( lambda: self.on_button_press(MovementType.Stop, False))
        
        
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