from .node import Node


class SingleThreadedExecutor:
    _rclpy_initialized = False

    def __init__(self) -> None:
        self.nodes: list[Node] = []
    
    def _init_executor(self):
        # Faking actual ROS rclpy init with a flag
        SingleThreadedExecutor._rclpy_initialized = True

    def add_node(self, node: Node) -> None:
        self.nodes.append(node)

    def spin(self) -> None:
        while True:
            # Deal with messages as fast as possible
            # Otherwise zmq keeps queueing them forever and eats ALL your memory
            self.spin_once()

    def spin_once(self) -> None:
        if not SingleThreadedExecutor._rclpy_initialized:
            raise RuntimeError('CWRUbotix tip: SingleThreadedExecutor.spin_once '
                               'was called before rclpy.init. '
                               'Run rclpy.init first!')

        for node in self.nodes:
            for subscription in node.subscriptions:
                subscription.secret_internal_poll()
