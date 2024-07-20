class PixhawkInstruction:
    MANUAL_CONTROL = 0
    KEYBOARD_CONTROL = 1
    AUTONOMOUS_CONTROL = 2

    def __init__(self, *,
                 forward: float = 0, lateral: float = 0, vertical: float = 0,
                 roll: float = 0, pitch: float = 0, yaw: float = 0,
                 author: int | None):

        if author is None:
            raise ValueError('Author cannot be none!')

        self.author = author

        self.forward = forward
        self.lateral = lateral
        self.vertical = vertical
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
