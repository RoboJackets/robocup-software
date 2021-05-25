import rclpy
from rclpy.node import Node

from rj_msgs import msg
import stp.rc as rc
import stp.utils.world_state_converter as conv
import stp.situation as situation
import stp.coordinator as coordinator
import numpy as np
from rj_gameplay.action.move import Move

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
        super().__init__('minimal_subscriber')
        self.world_state_sub = self.create_subscription(msg.WorldState, '/vision_filter/world_state', self.create_partial_world_state, 10)
        self.field_dimenstions = self.create_subscription(msg.FieldDimensions, '/config/field_dimensions', self.create_field, 10)
        self.game_info = self.create_subscription(msg.GameState, '/referee/game_state', self.create_game_info, 10)

        self.robot_state_subs = [None] * NUM_ROBOTS
        self.robot_intent_pubs = [None] * NUM_ROBOTS

        self.override_actions = [None] * NUM_ROBOTS

        for i in range(NUM_ROBOTS):
            self.robot_state_subs[i] = self.create_subscription(msg.RobotStatus, '/radio/robot_status/robot_'+str(i), self.create_partial_robots, 10)
 
        for i in range(NUM_ROBOTS):
            self.robot_intent_pubs[i] = self.create_publisher(msg.RobotIntent, '/gameplay/robot_intent/robot_'+str(i), 10)

        
        self.get_logger().info("Gameplay node started")
        self.world_state = world_state
        self.partial_world_state: conv.PartialWorldState = None
        self.game_info: rc.GameInfo = None
        self.field: rc.Field = None
        self.robot_statuses: List[conv.RobotStatus] = []

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
        if self.partial_world_state is not None and self.field is not None and len(self.robot_statuses) >= NUM_ROBOTS:
            #self.ger_logger().info("Robot Status Len: " +  str(len(self.robot_status)))
            #self.get_logger().info("Our Robots Len: " + str(len(self.partial_world_state.our_robots)))
            self.world_state = conv.worldstate_creator(self.partial_world_state, self.robot_statuses, self.game_info, self.field)

        return self.world_state

    def gameplay_tick(self) -> None:
        """
        ticks the gameplay coordinator using recent world_state
        """
        if self.partial_world_state is not None and self.field is not None and len(self.robot_statuses) >= NUM_ROBOTS:

            self.world_state = conv.worldstate_creator(self.partial_world_state, self.robot_statuses, self.game_info, self.field)

        #self.get_logger().info("Gameplay node is ticked")

        if self.world_state is not None:
            self.debug_async_lineup(self.world_state)
            self.tick_override_actions(self.world_state)
            # self.gameplay.tick(self.world_state)
            # Uncomment when a real play selector is created
        else:
            pass
            #self.get_logger().info("World state was none!")
    
    def tick_override_actions(self, world_state) -> None:
        for i in range(0,NUM_ROBOTS):
            if(self.override_actions[i] is not None):
                #self.get_logger().info("Intent published")
                fresh_intent = msg.RobotIntent()
                self.override_actions[i].tick(fresh_intent)
                self.robot_intent_pubs[i].publish(fresh_intent)

    def clear_override_actions(self) -> None:
        self.override_actions = [None] * NUM_ROBOTS

    def debug_async_lineup(self, world_state) -> None:

        left_x = -1.0
        right_x = 1.0
        start_y = 2.0
        y_inc = 0.3

        for i in range(len(self.override_actions)):
            if(self.override_actions[i] is not None):
                if(self.override_actions[i].is_done(world_state)):
                    if(self.override_actions[i].target_point[0] == left_x):
                        self.override_actions[i] = Move(robot_id = i, target_point=np.array([right_x,start_y + i * y_inc]))
                    else:
                        self.override_actions[i] = Move(robot_id = i, target_point=np.array([left_x,start_y + i * y_inc]))
            else:
                self.override_actions[i] = Move(robot_id = i, target_point=np.array([right_x,start_y + i * y_inc]))

    def debug_sync_lineup(self, world_state) -> None:

        left_x = -1.0
        right_x = 1.0
        start_y = 2.0
        y_inc = 0.4

        for i in range(len(self.override_actions)):
            if(self.override_actions[i] is None):
                self.override_actions[i] = Move(robot_id = i, target_point=np.array([left_x,start_y + i * y_inc]))

        done = True
        for i in range(len(self.override_actions)):
            if(not self.override_actions[i].is_done(world_state)):
                self.get_logger().info("move not done: " + str(i))
                done = False

        if(done):
            self.get_logger().info("Lineup done")
            left = self.override_actions[0].target_point[0] == left_x
            for i in range(len(self.override_actions)):
                if(left):
                    self.override_actions[i] = Move(robot_id = i, target_point=np.array([right_x,start_y + i * y_inc]))
                else:
                    self.override_actions[i] = Move(robot_id = i, target_point=np.array([left_x,start_y + i * y_inc]))


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
