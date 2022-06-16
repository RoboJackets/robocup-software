"""
ROS entry point into Python-side gameplay library.
Alternatively, where gameplay cedes control to cpp motion control/planning.

Contains TestPlaySelector, GameplayNode, and main() which spins GameplayNode
and allows the PlaySelector to be changed between Test and other forms.
"""

import importlib
from typing import List, Optional, Set

import numpy as np
import rclpy
import stp.local_parameters as local_parameters
import stp.play
import stp.rc as rc
import stp.situation as situation
import stp.skill
import stp.utils.world_state_converter as conv
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from rj_geometry_msgs import msg as geo_msg
from rj_msgs import msg
from rj_msgs.msg import EmptyMotionCommand, RobotIntent
from std_msgs.msg import String as StringMsg
from stp.action import IAction
from stp.global_parameters import GlobalParameterClient

import rj_gameplay.play_selector as play_selector

NUM_ROBOTS = 16


class GameplayNode(Node):
    """
    A node which subscribes to the world_state, game state, robot status, and
    field topics and converts the messages to python types.
    """

    def __init__(
        self,
        play_selector: situation.IPlaySelector,
        # TODO: see whether or not world_state is ever not None here
        world_state: Optional[rc.WorldState] = None,
    ) -> None:
        rclpy.init()
        super().__init__("gameplay_node")

        self._test_play = None
        self._curr_play = None
        self._curr_situation = None

        file = open("config/plays.txt")
        self.plays: Set = set(file.read().splitlines())
        file.close()

        # dynamically import plays based on name in config/plays.txt
        # https://stackoverflow.com/questions/44492803/dynamic-import-how-to-import-from-module-name-from-variable/44492879#44492879
        for play in self.plays:
            if play == "None":
                # None = our choice for "no test play"
                continue
            module_name = "rj_gameplay.play." + play[: play.find(".")]
            print(module_name)
            module = importlib.import_module(module_name)
            globals().update(
                {n: getattr(module, n) for n in module.__all__}
                if hasattr(module, "__all__")
                else {
                    k: v for (k, v) in module.__dict__.items() if not k.startswith("_")
                }
            )
            print(globals())

        self.declare_parameter("test_play", "None")

        self.add_on_set_parameters_callback(self.update_test_play)

        """uncomment the below line and use the local param server
        if we need a more robust param setup.
        Right now this is overengineering though:"""
        # local_parameters.register_parameters(self)

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

        self.test_play_sub = self.create_subscription(
            StringMsg, "test_play", self.set_test_play_callback, 1
        )

        self.play_selector: situation.IPlaySelector = play_selector

    def set_play_state(self, play_state: msg.PlayState):
        self.play_state = play_state

    def set_match_state(self, match_state: msg.MatchState):
        self.match_state = match_state

    def debug_callback(self):
        """
        Publishes the string that shows up in the behavior tree in the Soccer UI.
        """
        debug_text = ""
        debug_text += f"WorldState: {self.world_state}\n\n"
        debug_text += f"Play: {self._curr_play}\n\n"

        # situation is a long, ugly type: shorten before printing
        short_situation = f"{self._curr_situation}"
        short_situation = short_situation[short_situation.rfind(".") + 1 : -2]
        debug_text += f"Situation: {short_situation}\n\n"

        with np.printoptions(precision=3, suppress=True):
            for i, tactic in enumerate(self._curr_play.prioritized_tactics):
                debug_text += f"{i+1}. {tactic}\n\n"
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
        """
        Get situation, play from self.play_selector and update the currently running play if needed.
        Then, add field and game_info to world_state, and push global obstacles to motion planning.
        """
        self.update_world_state()

        raw_test_play_str = str(self.get_parameter("test_play").value)
        if raw_test_play_str != "None" or raw_test_play_str != "{0}.{1}".format(
            self._test_play.__class__.__module__, self._test_play.__class__.__name__
        ):
            play_str = raw_test_play_str[raw_test_play_str.find(".") + 1 :]
            self._test_play = eval(play_str)

        if self.world_state is not None:
            if self._test_play is None:
                new_situation, new_play = self.play_selector.select(self.world_state)

                # if play/situation hasn't changed, keep old play
                if type(self._curr_play) is not type(new_play) or type(
                    self._curr_situation
                ) != type(new_situation):
                    self._curr_play = new_play
                    self._curr_situation = new_situation
            else:
                self._curr_play = self._test_play

            intents = self._curr_play.tick(self.world_state)

            if intents:
                for i in range(len(self.world_state.our_robots)):
                    if intents[i] is not None:
                        # if intent given by gameplay, publish it
                        self.robot_intent_pubs[i].publish(intents[i])
                    else:
                        # otherwise, send empty (to stop previous intents)
                        empty_intent = RobotIntent()
                        empty_command = EmptyMotionCommand()
                        empty_intent.motion_command.empty_command = [empty_command]
                        self.robot_intent_pubs[i].publish(empty_intent)

            field = self.world_state.field
            game_info = self.build_game_info()

            def_area_obstacles = geo_msg.ShapeSet()
            self.add_def_areas_to_obs(def_area_obstacles, game_info)
            self.def_area_obstacles_pub.publish(def_area_obstacles)

            global_obstacles = geo_msg.ShapeSet()
            self.add_goals_to_global_obs(global_obstacles, game_info)
            self.add_ball_to_global_obs(global_obstacles, game_info)

            self.global_obstacles_pub.publish(global_obstacles)
            self.debug_callback()
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

    def update_test_play(self, parameter_list) -> SetParametersResult:
        """
        Checks all gameplay node parameters to see if they are valid.
        Right now test_play is the only one.
        It should be a string in the plays set.
        If we use the local param server later, move this logic there.
        """
        rejected_parameters = (
            param
            for param in parameter_list
            if param.name == "test_play"
            and param.type_ is Parameter.Type.STRING
            and str(param.value) not in self.plays
        )
        was_success = not any(rejected_parameters)
        return SetParametersResult(successful=(was_success))

    def set_test_play_callback(self, test_play_msg: StringMsg):
        new_test_play = rclpy.parameter.Parameter(
            "test_play", Parameter.Type.STRING, test_play_msg.data
        )
        self.set_parameters([new_test_play])

    def shutdown(self) -> None:
        """
        destroys node
        """
        self.destroy_node()
        rclpy.shutdown()


def main():
    my_play_selector = play_selector.PlaySelector()
    gameplay = GameplayNode(my_play_selector)
    rclpy.spin(gameplay)
