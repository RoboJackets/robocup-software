import rclpy
from rclpy.node import Node

from rj_msgs import msg
import stp.rc as rc
import stp.utils.world_state_converter as conv
import stp.situation as situation
import stp.coordinator as coordinator
import stp.local_parameters as local_parameters
from stp.global_parameters import GlobalParameterClient
import numpy as np

from typing import List, Optional

NUM_ROBOTS = 16

class EmptyPlaySelector(situation.IPlaySelector):
    # an empty play selector, replace with actual one when created

    def select(self, world_state: rc.WorldState) -> None:
        return None

class GameplayNode(Node):
    """
    A node which subscribes to the world_state,  game state, robot status, and field topics and converts the messages to python types.
    """

    def __init__(self, play_selector: situation.IPlaySelector, world_state: Optional[rc.WorldState] = None) -> None:
        rclpy.init()
        super().__init__('gameplay_node')
        self.world_state_sub = self.create_subscription(msg.WorldState, '/vision_filter/world_state', self.create_partial_world_state, 10)
        self.field_dimenstions = self.create_subscription(msg.FieldDimensions, '/config/field_dimensions', self.create_field, 10)
        self.game_info = self.create_subscription(msg.GameState, '/referee/game_state', self.create_game_info, 10)
        for i in range(NUM_ROBOTS):
            self.game_state_sub = self.create_subscription(msg.RobotStatus, '/radio/robot_status/robot_'+str(i), self.create_partial_robots, 10)

        self.world_state = world_state
        self.partial_world_state: conv.PartialWorldState = None
        self.game_info: rc.GameInfo = None
        self.field: rc.Field = None
        self.robot_statuses: List[conv.RobotStatus] = []

        self.global_parameter_client = GlobalParameterClient(
            self, '/global_parameter_server')
        local_parameters.register_parameters(self)

        timer_period = 1/60 #seconds
        self.timer = self.create_timer(timer_period, self.gameplay_tick)
        self.gameplay = coordinator.Coordinator(play_selector)


    def create_partial_world_state(self, msg: msg.WorldState) -> None:
        """
        Creates a partial world state from a world state message
        """
        if msg is not None:
            self.partial_world_state = conv.worldstate_message_converter(msg)

    def create_partial_robots(self, msg: msg.RobotStatus) -> None:
        """
        Creates the robot status which makes up part of the whole Robot class
        """
        if msg is not None:
            robot = conv.robotstatus_to_partial_robot(msg)
            index = robot.robot_id
            self.robot_statuses.insert(index, robot)

    def create_game_info(self, msg: msg.GameState) -> None:
        """
        Create game info object from Game State message
        """
        if msg is not None:
            self.game_info = conv.gamestate_to_gameinfo(msg)

    def create_field(self, msg: msg.FieldDimensions) -> None:
        """
        Creates field object from Field Dimensions message
        """
        if msg is not None:
            self.field = conv.field_msg_to_field(msg)


    def get_world_state(self) -> rc.WorldState:
        """
        returns: an updated world state
        """
        if self.partial_world_state is not None and self.field is not None and len(self.robot_statuses) >= 16:

            self.world_state = conv.worldstate_creator(self.partial_world_state, self.robot_statuses, self.game_info, self.field)

        return self.world_state

    def gameplay_tick(self) -> None:
        """
        ticks the gameplay coordinator using recent world_state
        """
        if self.partial_world_state is not None and self.field is not None:

            self.world_state = conv.worldstate_creator(self.partial_world_state, self.robot_statuses, self.game_info, self.field)

        if self.world_state is not None:
            pass
            # self.gameplay.tick(self.world_state)
            # Uncomment when a real play selector is created

    def shutdown(self) -> None:
        """
        destroys node
        """
        self.destroy_node()
        rclpy.shutdown()

def main():
    play_selector = EmptyPlaySelector()
    gameplay = GameplayNode(play_selector)
    rclpy.spin(gameplay)
