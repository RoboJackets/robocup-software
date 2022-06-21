import os
from concurrent.futures import ThreadPoolExecutor
from typing import List

from rclpy.executors import Executor
from rclpy.node import Node


class GameplayExecutor(Executor):
    """
    Custom ROS2 executor for gameplay.
    Execute a solo node in a single thread (gameplay_node)
    and set all other nodes (the clients) to share group of threads
    """

    def __init__(self):
        super().__init__()
        self.pool_nodes = set()
        self.solo_node = None
        self.pool_executor = ThreadPoolExecutor(4 or os.cpu_count())
        self.solo_executor = ThreadPoolExecutor(max_workers=1)

    def add_solo_node(self, node: Node):
        self.solo_node = node
        # add_node inherited
        self.add_node(node)

    def add_pool_nodes(self, nodes: List[Node]):
        for node in nodes:
            self.pool_nodes.add(node)
            # add_node inherited
            self.add_node(node)

    def spin_once(self, timeout_sec=None):
        """
        Execute a single callback, then return.
        This is the only function which must be overridden by a custom executor. Its job is to
        start executing one callback, then return. It uses the method `wait_for_ready_callbacks`
        to get work to execute.
        :param timeout_sec: Seconds to wait. Block forever if None. Don't wait if <= 0
        :type timeout_sec: float or None
        """
        # wait_for_ready_callbacks yields callbacks that are ready to be executed
        try:
            handler, group, node = self.wait_for_ready_callbacks(
                timeout_sec=timeout_sec
            )
        except StopIteration:
            pass
        else:
            if node is self.solo_node:
                self.solo_executor.submit(handler)
            else:
                self.pool_executor.submit(handler)
