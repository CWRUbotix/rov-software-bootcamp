from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton
from enum import Enum
from bootcamp_harness.rclpy.node import Node
from bootcamp_harness.rclpy.qos import QoSPresetProfiles
from bootcamp_harness.rov_msgs.msg import PixhawkInstruction
from bootcamp_harness import rclpy
from pynput.keyboard import Key, Listener, KeyCode

class MovementType(Enum):
    Forward = 1
    Vertical = 2
    Lateral = 3
    Pitch = 4
    Yaw = 5
    Stop = 6

# class ButtonPanel(QWidget):
#     def __init__(self) -> None:
#         super().__init__()

#         node = Node('pixhawk_publisher')
#         self.pixhawk_publisher = node.create_publisher(
#             PixhawkInstruction,
#             'pixhawk_control',
#             QoSPresetProfiles.DEFAULT.value
#         )

#         layout = QVBoxLayout()
#         self.setLayout(layout)

#         top_layout = QHBoxLayout()
#         layout.addLayout(top_layout)
        
#         bottom_layout = QHBoxLayout()
#         layout.addLayout(bottom_layout)

        

#         self.add_button(top_layout, 'Up', MovementType.Vertical, True)
#         self.add_button(top_layout, '↰', MovementType.Yaw, False)
#         self.add_button(top_layout, '↑', MovementType.Forward, True)
#         self.add_button(top_layout, '↱', MovementType.Yaw, True)
#         self.add_button(top_layout, '/', MovementType.Pitch, True)
#         self.add_button(top_layout, 'Stop', MovementType.Stop, True)
#         self.add_button(bottom_layout, 'Down', MovementType.Vertical, False)
#         self.add_button(bottom_layout, '←', MovementType.Lateral, False)
#         self.add_button(bottom_layout, '↓', MovementType.Forward, False)
#         self.add_button(bottom_layout, '→', MovementType.Lateral, True)
#         self.add_button(bottom_layout, '\\', MovementType.Pitch, False)
#         self.add_button(bottom_layout, '', MovementType.Stop, False)

#     def add_button(self, layout: QHBoxLayout, name, movement_type: MovementType, direction: bool):
#         button = QPushButton(name)
#         button.clicked.connect(
#             lambda: self.on_button_press(movement_type, direction)
#         )
#         layout.addWidget(button)
    
#     def on_button_press(self, movement_type: MovementType, direction: bool):
#         value = 0.5 if direction else -0.5

#         instruction = PixhawkInstruction(
#             forward=(value if movement_type == MovementType.Forward else 0),
#             vertical=(value if movement_type == MovementType.Vertical else 0),
#             lateral=(value if movement_type == MovementType.Lateral else 0),
#             pitch=(value if movement_type == MovementType.Pitch else 0),
#             yaw=(value if movement_type == MovementType.Yaw else 0),
#             author=PixhawkInstruction.MANUAL_CONTROL
#         )
#         self.pixhawk_publisher.publish(instruction)
    
#         direction_str = 'positively' if direction else 'negatively'
#         print(f'Moving: {movement_type.name} {direction_str}')

class KeyboardPanel(QWidget):
    def __init__(self) -> None:
        super().__init__()

        # Start Keyboard Listener
        listener = Listener(
            on_press=self.on_press
        )
        listener.start()

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

        self.add_button(top_layout, 'Up', MovementType.Vertical, True)
        self.add_button(top_layout, '↰', MovementType.Yaw, False)
        self.add_button(top_layout, '↑', MovementType.Forward, True)
        self.add_button(top_layout, '↱', MovementType.Yaw, True)
        self.add_button(top_layout, '/', MovementType.Pitch, True)
        self.add_button(top_layout, 'Stop', MovementType.Stop, True)
        self.add_button(bottom_layout, 'Down', MovementType.Vertical, False)
        self.add_button(bottom_layout, '←', MovementType.Lateral, False)
        self.add_button(bottom_layout, '↓', MovementType.Forward, False)
        self.add_button(bottom_layout, '→', MovementType.Lateral, True)
        self.add_button(bottom_layout, '\\', MovementType.Pitch, False)
        self.add_button(bottom_layout, '', MovementType.Stop, False)

    def add_button(self, layout: QHBoxLayout, name, movement_type: MovementType, direction: bool):
        button = QPushButton(name)
        button.clicked.connect(
            lambda: self.on_button_press(movement_type, direction)
        )
        layout.addWidget(button)
    
    def on_press(self, key):
        if key == KeyCode.from_char('w'): 
            self.on_button_press(MovementType.Forward, True)
        if key == KeyCode.from_char('s'):
            self.on_button_press(MovementType.Forward, False)
        if key == KeyCode.from_char('a'):
            self.on_button_press(MovementType.Lateral, False)
        if key == KeyCode.from_char('d'):
            self.on_button_press(MovementType.Lateral, True)
        if key == Key.up:
            self.on_button_press(MovementType.Vertical, True)
        if key == Key.down:
            self.on_button_press(MovementType.Vertical, False)
        if key == Key.left:
            self.on_button_press(MovementType.Yaw, False)
        if key == Key.right:
            self.on_button_press(MovementType.Yaw, True)
        if key == Key.space:
            self.on_button_press(MovementType.Stop, True)
        if key == Key.esc:
            # Stop listener
            return False

    def on_button_press(self, movement_type: MovementType, direction: bool):
        value = 0.5 if direction else -0.5

        instruction = PixhawkInstruction(
            forward=(value if movement_type == MovementType.Forward else 0),
            vertical=(value if movement_type == MovementType.Vertical else 0),
            lateral=(value if movement_type == MovementType.Lateral else 0),
            pitch=(value if movement_type == MovementType.Pitch else 0),
            yaw=(value if movement_type == MovementType.Yaw else 0),
            author=PixhawkInstruction.MANUAL_CONTROL
        )
        self.pixhawk_publisher.publish(instruction)
    
        direction_str = 'positively' if direction else 'negatively'
        print(f'Moving: {movement_type.name} {direction_str}')

def main():
    rclpy.init()
    app = QApplication([])

    window = KeyboardPanel()
    window.show()

    app.exec()

if __name__ == '__main__':
    main()