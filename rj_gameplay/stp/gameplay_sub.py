import rclpy
from rclpy.node import Node

from rj_msgs import msg
import stp.rc as rc
from stp.utils.world_state_converter import worldstate_message_converter
import numpy as np

from typing import List, Optional


class GameplaySub(Node):
    """A node which subscribes to the world_state and game state topics and creates the converts the messages to python types."""

    def __init__(self, world_state: Optional[rc.WorldState] = None) -> None:
        rclpy.init()
        super().__init__('minimal_subscriber')
        self.world_state_sub = self.create_subscription(msg.WorldState, '/vision_filter/world_state', self.update_world_state, 10)
        self.game_state_sub = self.create_subscription(msg.GameState, '/game_state', self.update_game_state, 10)
        self.world_state = world_state

    def update_world_state(self, msg: msg.WorldState) -> None:
        self.world_state = worldstate_message_converter(msg)

    def update_game_state(self, msg: msg.GameState) -> None:
        ...
        # Implenet when Game State message has a publisher

    def get_world_state(self) -> rc.WorldState:
        """
        returns: an updated world state
        """
        rclpy.spin_once(self)
        return self.world_state

    def shutdown(self) -> None:
        """
        destroys node
        """
        self.destroy_node()
        rclpy.shutdown()