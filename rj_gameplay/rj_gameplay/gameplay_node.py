import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from rj_msgs import msg
from rj_geometry_msgs import msg as geo_msg
import stp.rc as rc
import stp.utils.world_state_converter as conv
import stp.situation as situation
import stp.coordinator as coordinator
import stp
import stp.skill
import stp.play
import stp.local_parameters as local_parameters
from stp.global_parameters import GlobalParameterClient
import numpy as np
from rj_gameplay.action.move import Move
from rj_gameplay.play import basic_defense, basic_scramble, passing_tactic_play, defend_restart, restart, kickoff_play, \
    basic122, penalty_defense
from typing import List, Optional, Tuple
from std_msgs.msg import String as StringMsg

import stp.basic_play_selector as basic_play_selector

NUM_ROBOTS = 16


class EmptyPlaySelector(situation.IPlaySelector):
    # an empty play selector, replace with actual one when created

    def select(self, world_state: rc.WorldState) -> None:
        return None


class TestPlaySelector(situation.IPlaySelector):
    def select(self, world_state: rc.WorldState) -> Tuple[situation.ISituation, stp.play.IPlay]:
        self.curr_situation = None
        return (None, penalty_defense.PenaltyDefense())


class GameplayNode(Node):
    """
    A node which subscribes to the world_state, game state, robot status, and field topics and converts the messages to python types.
    """

    def __init__(self, play_selector: situation.IPlaySelector, world_state: Optional[rc.WorldState] = None) -> None:
        rclpy.init()
        super().__init__('gameplay_node')
        self.world_state_sub = self.create_subscription(msg.WorldState, 'vision_filter/world_state',
                                                        self.create_partial_world_state, 10)
        self.field_dimensions = self.create_subscription(msg.FieldDimensions, 'config/field_dimensions',
                                                         self.create_field, 10)

        self.play_state = None
        self.match_state = None
        self.play_state_sub = self.create_subscription(msg.PlayState, 'referee/play_state', self.set_play_state, 10)
        self.match_state_sub = self.create_subscription(msg.MatchState, 'referee/match_state', self.set_match_state, 10)

        keep_latest = QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        self.goalie_id_sub = self.create_subscription(msg.Goalie,
                                                      'referee/our_goalie',
                                                      self.create_goalie_id,
                                                      keep_latest)

        self.robot_state_subs = [None] * NUM_ROBOTS
        self.robot_intent_pubs = [None] * NUM_ROBOTS

        self.override_actions = [None] * NUM_ROBOTS

        for i in range(NUM_ROBOTS):
            self.robot_state_subs[i] = self.create_subscription(msg.RobotStatus, 'radio/robot_status/robot_' + str(i),
                                                                self.create_partial_robots, 10)

        for i in range(NUM_ROBOTS):
            self.robot_intent_pubs[i] = self.create_publisher(msg.RobotIntent, 'gameplay/robot_intent/robot_' + str(i),
                                                              10)

        self.get_logger().info("Gameplay node started")
        self.world_state = world_state
        self.partial_world_state: conv.PartialWorldState = None
        self.goalie_id = None
        self.field: rc.Field = None
        self.robot_statuses: List[conv.RobotStatus] = [conv.RobotStatus()] * NUM_ROBOTS * 2
        self.ball_placement = None

        self.global_parameter_client = GlobalParameterClient(
            self, 'global_parameter_server')
        local_parameters.register_parameters(self)

        # publish global obstacles
        self.goal_zone_obstacles_pub = self.create_publisher(geo_msg.ShapeSet, 'planning/goal_zone_obstacles', 10)
        self.global_obstacles_pub = self.create_publisher(geo_msg.ShapeSet, 'planning/global_obstacles', 10)

        timer_period = 1 / 60  # seconds
        self.timer = self.create_timer(timer_period, self.gameplay_tick)

        self.debug_text_pub = self.create_publisher(StringMsg,
                                                    '/gameplay/debug_text', 10)
        self.play_selector = play_selector
        self.gameplay = coordinator.Coordinator(play_selector,
                                                self.debug_callback)

    def set_play_state(self, play_state: msg.PlayState):
        self.play_state = play_state

    def set_match_state(self, match_state: msg.MatchState):
        self.match_state = match_state

    def debug_callback(self, play: stp.play.IPlay, skills):
        debug_text = ""
        debug_text += f"{type(play).__name__}({type(self.play_selector.curr_situation).__name__})\n"
        with np.printoptions(precision=3, suppress=True):
            for skill in skills:
                debug_text += f"  {skill}\n"
        self.debug_text_pub.publish(StringMsg(data=debug_text))

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
            self.robot_statuses[index] = robot

    def build_game_info(self) -> Optional[rc.GameInfo]:
        """
        Create game info object from Game State message
        """
        if self.play_state is not None and self.match_state is not None:
            return conv.build_game_info(self.play_state, self.match_state)
        return None

    def create_field(self, msg: msg.FieldDimensions) -> None:
        """
        Creates field object from Field Dimensions message
        """
        if msg is not None:
            self.field = conv.field_msg_to_field(msg)

    def create_goalie_id(self, msg: msg.Goalie) -> None:
        """
        Set game_info's goalie_id based on goalie msg
        """
        self.goalie_id = msg.goalie_id

    def get_world_state(self) -> rc.WorldState:
        """
        returns: an updated world state
        """
        if self.partial_world_state is not None and self.field is not None:
            self.world_state = conv.worldstate_creator(self.partial_world_state, self.robot_statuses,
                                                       self.build_game_info(),
                                                       self.field, self.goalie_id)

        return self.world_state

    def gameplay_tick(self) -> None:
        """
        ticks the gameplay coordinator using recent world_state
        """

        if self.partial_world_state is not None and self.field is not None and len(self.robot_statuses) >= NUM_ROBOTS:
            self.world_state = conv.worldstate_creator(self.partial_world_state, self.robot_statuses,
                                                       self.build_game_info(),
                                                       self.field, self.goalie_id)
        else:
            self.world_state = None

        if self.world_state is not None:
            intents = self.gameplay.tick(self.world_state)
            for i in range(NUM_ROBOTS):
                self.robot_intent_pubs[i].publish(intents[i])

            # create our_penalty rect
            our_penalty = geo_msg.Rect()
            top_left = geo_msg.Point(x=self.field.penalty_long_dist_m / 2 + self.field.line_width_m, y=0.0)
            bot_right = geo_msg.Point(x=-self.field.penalty_long_dist_m / 2 - self.field.line_width_m,
                                      y=self.field.penalty_short_dist_m)
            our_penalty.pt = [top_left, bot_right]

            # create their_penalty rect
            # add distance slack for stops (0.2 min)
            # https://robocup-ssl.github.io/ssl-rules/sslrules.html#_robot_too_close_to_opponent_defense_area
            game_info = self.build_game_info()
            add_stop_dist = game_info is None or game_info.state == rc.GameState.STOP or game_info.is_restart()
            DIST_FOR_STOP = 0.3 if add_stop_dist else 0.0  # > 0.2 m

            their_penalty = geo_msg.Rect()
            left_x = self.field.penalty_long_dist_m / 2 + self.field.line_width_m + DIST_FOR_STOP
            bot_left = geo_msg.Point(x=left_x, y=self.field.length_m)
            top_right = geo_msg.Point(x=-left_x, y=self.field.length_m - (
                    self.field.penalty_short_dist_m + self.field.line_width_m + DIST_FOR_STOP))
            their_penalty.pt = [bot_left, top_right]

            our_goal = geo_msg.Rect(pt=[
                geo_msg.Point(x=-self.field.goal_width_m / 2, y=0.),
                geo_msg.Point(x=self.field.goal_width_m / 2, y=-self.field.goal_depth_m),
            ])
            their_goal = geo_msg.Rect(pt=[
                geo_msg.Point(x=-self.field.goal_width_m / 2, y=self.field.length_m),
                geo_msg.Point(x=self.field.goal_width_m / 2, y=self.field.length_m+self.field.goal_depth_m),
            ])

            global_obstacles = geo_msg.ShapeSet()
            global_obstacles.rectangles = [our_goal, their_goal]
            if game_info is not None:
                ball_point = self.world_state.ball.pos
                if game_info.is_stopped() or game_info.their_restart and (game_info.is_indirect() or game_info.is_direct()):
                    global_obstacles.circles.append(
                        geo_msg.Circle(center=geo_msg.Point(x=ball_point[0], y=ball_point[1]), radius=0.6))
                if game_info.is_kickoff() and game_info.their_restart:
                    global_obstacles.circles.append(
                        geo_msg.Circle(center=geo_msg.Point(x=ball_point[0], y=ball_point[1]), radius=0.3))
                if game_info.is_kickoff() and game_info.is_setup() and game_info.our_restart:
                    global_obstacles.circles.append(
                        geo_msg.Circle(center=geo_msg.Point(x=ball_point[0], y=ball_point[1]), radius=0.1))
                if game_info.is_free_placement():
                    for t in np.linspace(0.0, 1.0, 20):
                        placement = game_info.ball_placement()

                        pt = ball_point * t + (1 - t) * placement
                        global_obstacles.circles.append(
                            geo_msg.Circle(center=geo_msg.Point(x=pt[0], y=pt[1]), radius=0.8))
            self.global_obstacles_pub.publish(global_obstacles)

            # publish Rect shape to goal_zone_obstacles topic
            goal_zone_obstacles = geo_msg.ShapeSet()
            goal_zone_obstacles.rectangles = [our_penalty, their_penalty]
            self.goal_zone_obstacles_pub.publish(goal_zone_obstacles)
        else:
            self.get_logger().warn("World state was none!")

    def tick_override_actions(self, world_state) -> None:
        for i in range(0, NUM_ROBOTS):
            if self.override_actions[i] is not None:
                fresh_intent = msg.RobotIntent()
                self.override_actions[i].tick(fresh_intent)
                self.robot_intent_pubs[i].publish(fresh_intent)

    def clear_override_actions(self) -> None:
        self.override_actions = [None] * NUM_ROBOTS

    def shutdown(self) -> None:
        """
        destroys node
        """
        self.destroy_node()
        rclpy.shutdown()


def main():
    # play_selector = TestPlaySelector()
    play_selector = basic_play_selector.BasicPlaySelector()
    gameplay = GameplayNode(play_selector)
    rclpy.spin(gameplay)
