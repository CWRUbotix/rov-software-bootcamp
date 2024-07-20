from time import sleep

from bootcamp_harness.rclpy.node import Node


#### INTERNAL INIT FUNCTIONS ####


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
            self.spin_once()
            sleep(0.1)

    def spin_once(self) -> None:
        if not SingleThreadedExecutor._rclpy_initialized:
            raise RuntimeError('CWRUbotix tip: SingleThreadedExecutor.spin_once'
                               'was called before rclpy.init.'
                               'Run rclpy.init first!')

        for node in self.nodes:
            for subscription in node.subscriptions:
                subscription.secret_internal_poll()


__global_executor: SingleThreadedExecutor = SingleThreadedExecutor()

def __get_global_executor() -> SingleThreadedExecutor:
    global __global_executor
    return __global_executor

def __init():
    """
    Initialize ROS communications for a given context.
    """
    __get_global_executor()._init_executor()

def __spin(node: Node, executor: SingleThreadedExecutor | None = None):
    """
    Execute work and block until the context associated with the executor is shutdown.

    Callbacks will be executed by the provided executor.

    This function blocks.

    :param node: A node to add to the executor to check for work.
    :param executor: The executor to use, or the global executor if ``None``.
    """
    if not SingleThreadedExecutor._rclpy_initialized:
        raise RuntimeError('CWRUbotix tip: rclpy.spin was called'
                           'before rclpy.init. Run rclpy.init first!')

    chosen_executor = __get_global_executor() if executor is None else executor
    chosen_executor.add_node(node)
    chosen_executor.spin()
    print('spinning globally')
