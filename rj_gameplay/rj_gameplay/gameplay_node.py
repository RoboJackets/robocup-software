"""
ROS entry point into Python-side gameplay library.
Alternatively, where gameplay cedes control to cpp motion control/planning.

Contains TestPlaySelector, GameplayNode, and main() which spins GameplayNode
and allows the PlaySelector to be changed between Test and other forms.
"""

from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from rj_msgs import msg
from rj_geometry_msgs import msg as geo_msg
from std_msgs.msg import String as StringMsg

import stp.rc as rc
import stp.utils.world_state_converter as conv
import stp.situation as situation
import stp.skill
import stp.play
from stp.action import IAction

import stp.local_parameters as local_parameters
from stp.global_parameters import GlobalParameterClient

import numpy as np
from rj_gameplay.action.move import Move

# ignore "unused import" error
from rj_gameplay.play import line_up, basic_defense, keepaway, basic122  # noqa: F401
import rj_gameplay.basic_play_selector as basic_play_selector
from stp.utils.constants import RobotConstants

NUM_ROBOTS = RobotConstants.NUM_ROBOTS


class GameplayNode(Node):
    """
    A node which subscribes to the world_state, game state, robot status, and
    field topics and converts the messages to python types.
    """

    def __init__(
        self,
        play_selector: situation.IPlaySelector,
        test_play: Optional[stp.play.Play] = None,
        # TODO: see whether or not world_state is ever not None here
        world_state: Optional[rc.WorldState] = None,
    ) -> None:
        rclpy.init()
        super().__init__("gameplay_node")

        # do not change this line, change the test play passed in at bottom of file
        self.test_play = test_play

        self.world_state_sub = self.create_subscription(
            msg.WorldState,
            "vision_filter/world_state",
            self.create_partial_world_state,
            10,
        )
        self.field_dimensions = self.create_subscription(
            msg.FieldDimensions,
            "config/field_dimensions",
            self.create_field,
            10,
        )

        self.play_state = None
        self.match_state = None
        self.play_state_sub = self.create_subscription(
            msg.PlayState, "referee/play_state", self.set_play_state, 10
        )
        self.match_state_sub = self.create_subscription(
            msg.MatchState, "referee/match_state", self.set_match_state, 10
        )

        keep_latest = QoSProfile(
            depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.goalie_id_sub = self.create_subscription(
            msg.Goalie,
            "referee/our_goalie",
            self.create_goalie_id,
            keep_latest,
        )

        self.override_actions: List[Optional[IAction]] = [None] * NUM_ROBOTS

        # create lists for robot state subs and robot intent pubs (ROS)
        self.robot_state_subs = []
        for i in range(NUM_ROBOTS):
            self.robot_state_subs.append(
                self.create_subscription(
                    msg.RobotStatus,
                    "radio/robot_status/robot_" + str(i),
                    self.create_partial_robots,
                    10,
                )
            )

        self.robot_intent_pubs = []
        for i in range(NUM_ROBOTS):
            self.robot_intent_pubs.append(
                self.create_publisher(
                    msg.RobotIntent,
                    "gameplay/robot_intent/robot_" + str(i),
                    10,
                )
            )

        self.get_logger().info("Gameplay node started")
        self.world_state = world_state
        # these 3 Nones will be filled in dynamically
        self.partial_world_state: Optional[conv.PartialWorldState] = None
        self.goalie_id: Optional[int] = None
        self.field: Optional[rc.Field] = None
        self.robot_statuses: List[conv.RobotStatus] = (
            [conv.RobotStatus()] * NUM_ROBOTS * 2
        )
        self.ball_placement = None

        self.global_parameter_client = GlobalParameterClient(
            self, "global_parameter_server"
        )
        local_parameters.register_parameters(self)

        # publish def_area_obstacles, global obstacles
        self.def_area_obstacles_pub = self.create_publisher(
            geo_msg.ShapeSet, "planning/def_area_obstacles", 10
        )
        self.global_obstacles_pub = self.create_publisher(
            geo_msg.ShapeSet, "planning/global_obstacles", 10
        )

        timer_period = 1 / 60  # seconds
        self.timer = self.create_timer(timer_period, self.gameplay_tick)

        self.debug_text_pub = self.create_publisher(
            StringMsg, "/gameplay/debug_text", 10
        )
        self.play_selector: situation.IPlaySelector = play_selector

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

    def update_world_state(self) -> None:
        """
        returns: an updated world state
        """
        if (
            self.partial_world_state is not None
            and self.field is not None
            and self.goalie_id is not None
        ):
            self.world_state = conv.worldstate_creator(
                self.partial_world_state,
                self.robot_statuses,
                self.build_game_info(),
                self.field,
                self.goalie_id,
            )
            assert self.world_state is not None

    def gameplay_tick(self) -> None:
        self.update_world_state()

        if self.world_state is not None:
            if self.test_play is None:
                curr_situation, curr_play = self.play_selector.select(self.world_state)
                intents = curr_play.tick(self.world_state)
            else:
                intents = self.test_play.tick(self.world_state)

            if intents:
                for i in range(len(self.world_state.our_robots)):
                    if intents[i] is not None:
                        rip_i = self.robot_intent_pubs[i]
                        rip_i.publish(intents[i])

            field = self.world_state.field
            game_info = self.build_game_info()

            def_area_obstacles = geo_msg.ShapeSet()
            self.add_def_areas_to_obs(def_area_obstacles, game_info)
            self.def_area_obstacles_pub.publish(def_area_obstacles)

            global_obstacles = geo_msg.ShapeSet()
            self.add_goals_to_global_obs(global_obstacles, game_info)
            self.add_ball_to_global_obs(global_obstacles, game_info)

            self.global_obstacles_pub.publish(global_obstacles)

        else:
            self.get_logger().warn("World state was none!")

    def add_def_areas_to_obs(self, def_area_obstacles, game_info) -> None:
        """Creates and publishes rectangles for the defense area in front of both goals.

        The defense area, per the rules, is the box in front of each goal where
        only that team's goalie can be in and touch the ball.

        (Formerly referred to as "goal_zone_obstacles".)
        """
        # this is only ever called when self.field is filled in
        assert self.field is not None

        # create Rect for our def_area box
        our_def_area = geo_msg.Rect()
        top_left = geo_msg.Point(
            x=self.field.def_area_long_dist_m / 2 + self.field.line_width_m,
            y=0.0,
        )
        bot_right = geo_msg.Point(
            x=-self.field.def_area_long_dist_m / 2 - self.field.line_width_m,
            y=self.field.def_area_short_dist_m,
        )
        our_def_area.pt = [top_left, bot_right]

        # slack for distance (m) in Stop situations
        # https://robocup-ssl.github.io/ssl-rules/sslrules.html#_robot_too_close_to_opponent_defense_area
        add_stop_dist = (
            game_info is None
            or game_info.state == rc.GameState.STOP
            or (game_info.is_restart() and not game_info.is_penalty())
        )
        DIST_FOR_STOP = 0.3 if add_stop_dist else 0.0

        # create Rect for their def_area box
        their_def_area = geo_msg.Rect()
        left_x = (
            self.field.def_area_long_dist_m / 2
            + self.field.line_width_m
            + DIST_FOR_STOP
        )
        bot_left = geo_msg.Point(x=left_x, y=self.field.length_m)
        top_right = geo_msg.Point(
            x=-left_x,
            y=self.field.length_m
            - (
                self.field.def_area_short_dist_m
                + self.field.line_width_m
                + DIST_FOR_STOP
            ),
        )
        their_def_area.pt = [bot_left, top_right]

        # publish Rect shape to def_area_obstacles topic
        def_area_obstacles.rectangles = [our_def_area, their_def_area]

    def add_goals_to_global_obs(self, global_obstacles, game_info):
        """Adds the physical walls that form each goal to global_obstacles."""
        # this is only ever called when self.field is filled in
        assert self.field is not None

        physical_goal_board_width = 0.1
        our_goal = [
            geo_msg.Rect(
                pt=[
                    geo_msg.Point(
                        x=self.field.goal_width_m / 2,
                        y=-self.field.goal_depth_m,
                    ),
                    geo_msg.Point(
                        x=-self.field.goal_width_m / 2,
                        y=-self.field.goal_depth_m - physical_goal_board_width,
                    ),
                ]
            ),
            geo_msg.Rect(
                pt=[
                    geo_msg.Point(
                        x=self.field.goal_width_m / 2,
                        y=-self.field.goal_depth_m,
                    ),
                    geo_msg.Point(
                        x=self.field.goal_width_m / 2 + physical_goal_board_width,
                        y=0.0,
                    ),
                ]
            ),
            geo_msg.Rect(
                pt=[
                    geo_msg.Point(
                        x=-self.field.goal_width_m / 2,
                        y=-self.field.goal_depth_m,
                    ),
                    geo_msg.Point(
                        x=-self.field.goal_width_m / 2 - physical_goal_board_width,
                        y=0.0,
                    ),
                ]
            ),
        ]
        their_goal = [
            geo_msg.Rect(
                pt=[
                    geo_msg.Point(
                        x=self.field.goal_width_m / 2,
                        y=self.field.length_m + self.field.goal_depth_m,
                    ),
                    geo_msg.Point(
                        x=-self.field.goal_width_m / 2,
                        y=self.field.length_m
                        + self.field.goal_depth_m
                        + physical_goal_board_width,
                    ),
                ]
            ),
            geo_msg.Rect(
                pt=[
                    geo_msg.Point(
                        x=self.field.goal_width_m / 2,
                        y=self.field.length_m + self.field.goal_depth_m,
                    ),
                    geo_msg.Point(
                        x=self.field.goal_width_m / 2 + physical_goal_board_width,
                        y=self.field.length_m,
                    ),
                ]
            ),
            geo_msg.Rect(
                pt=[
                    geo_msg.Point(
                        x=-self.field.goal_width_m / 2,
                        y=self.field.length_m + self.field.goal_depth_m,
                    ),
                    geo_msg.Point(
                        x=-self.field.goal_width_m / 2 - physical_goal_board_width,
                        y=self.field.length_m,
                    ),
                ]
            ),
        ]

        global_obstacles.rectangles = our_goal + their_goal

    def add_ball_to_global_obs(self, global_obstacles, game_info):
        """
        Adds circular no-fly zone around ball during stops or restarts,
        to comply with rulebook.
        """

        # this is only ever called when self.field is filled in
        assert self.field is not None

        if game_info is not None:
            ball_point = self.world_state.ball.pos
            if (
                game_info.is_stopped()
                or game_info.their_restart
                and (game_info.is_indirect() or game_info.is_direct())
            ):
                global_obstacles.circles.append(
                    geo_msg.Circle(
                        center=geo_msg.Point(x=ball_point[0], y=ball_point[1]),
                        radius=0.6,
                    )
                )
            if game_info.is_kickoff() and game_info.their_restart:
                global_obstacles.circles.append(
                    geo_msg.Circle(
                        center=geo_msg.Point(x=ball_point[0], y=ball_point[1]),
                        radius=0.3,
                    )
                )
            if (
                game_info.is_kickoff()
                and game_info.is_setup()
                and game_info.our_restart
            ):
                global_obstacles.circles.append(
                    geo_msg.Circle(
                        center=geo_msg.Point(x=ball_point[0], y=ball_point[1]),
                        radius=0.1,
                    )
                )
            if game_info.is_free_placement():
                for t in np.linspace(0.0, 1.0, 20):
                    placement = game_info.ball_placement()

                    pt = ball_point * t + (1 - t) * placement
                    global_obstacles.circles.append(
                        geo_msg.Circle(
                            center=geo_msg.Point(x=pt[0], y=pt[1]), radius=0.8
                        )
                    )

    def tick_override_actions(self, world_state) -> None:
        for i in range(0, NUM_ROBOTS):
            oa_i = self.override_actions[i]
            if oa_i is not None:
                fresh_intent = msg.RobotIntent()
                oa_i.tick(fresh_intent)
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
    play_selector = basic_play_selector.BasicPlaySelector()

    # change this line to test different plays (set to None if no desired test play)

    test_play = basic122.Basic122()

    gameplay = GameplayNode(play_selector, test_play)
    rclpy.spin(gameplay)
