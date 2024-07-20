from bootcamp_harness.rclpy.executors import SingleThreadedExecutor
from bootcamp_harness.rclpy.node import Node

__global_executor: SingleThreadedExecutor = SingleThreadedExecutor()

def get_global_executor() -> SingleThreadedExecutor:
    global __global_executor
    return __global_executor

def init():
    """
    Initialize ROS communications for a given context.
    """
    get_global_executor()._init_executor()

def spin(node: Node, executor: SingleThreadedExecutor | None = None):
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

    chosen_executor = get_global_executor() if executor is None else executor
    chosen_executor.add_node(node)
    chosen_executor.spin()
    print('spinning globally')
