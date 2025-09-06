class PixhawkInstruction:
    MANUAL_CONTROL = 0
    KEYBOARD_CONTROL = 1
    AUTONOMOUS_CONTROL = 2

    def __init__(self,
                 x: float = 0, y: float = 0, z: float = 0, 
                 yaw: float = 0, pitch: float = 0, roll: float = 0,
                 author: int | None = None) -> None:

        self.author = author
        self.x = x
        self.y = y
        self.a = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def __str__(self) -> str:
        return (
            f'forward: {self.x}, lateral: {self.y}, vertical: {self.z}, '
            f'roll: {self.roll}, pitch: {self.pitch}, yaw: {self.yaw}, '
            f'author: {self.author}'
        )
