from bootcamp_harness.rclpy.executors import SingleThreadedExecutor, __get_global_executor, __init, __spin
from bootcamp_harness.rclpy.node import Node

def get_global_executor():
    return __get_global_executor()

def init():
    return __init()

def spin(node: Node, executor: SingleThreadedExecutor | None = None):
    """
    Execute work and block until the context associated with the executor is shutdown.

    Callbacks will be executed by the provided executor.

    This function blocks.

    :param node: A node to add to the executor to check for work.
    :param executor: The executor to use, or the global executor if ``None``.
    """
    return __spin(node, executor)
