from rj_msgs import msg
import stp.rc as rc
import numpy as np

from typing import List, Optional

RobotId = Optional[int]


class RobotStatus:
    """
    A class to contain the information from the robot status messsage
    """

    __slots = [
        "robot_id",
        "has_ball_sense",
        "kicker_charged",
        "kicker_healthy",
        "lethal_fault",
    ]

    robot_id: RobotId
    """
    visible: bool
    has_ball_sense: bool
    kicker_charged: bool
    kicker_healthy: bool
    lethal_fault: bool
    """

    def __init__(
        self,
        robot_id: RobotId = None,
        has_ball_sense: bool = None,
        kicker_charged: bool = None,
        kicker_healthy: bool = None,
        lethal_fault: bool = None,
    ):
        self.robot_id = robot_id
        self.has_ball_sense = has_ball_sense
        self.kicker_charged = kicker_charged
        self.kicker_healthy = kicker_healthy
        self.lethal_fault = lethal_fault


class RobotState:
    """
    A class to contain the infomration from the robot state message
    """

    __slots = ["id", "pose", "twist", "visible"]

    id: RobotId
    pose: np.ndarray
    twist: np.ndarray
    visible: bool

    def __init__(
        self,
        id: RobotId,
        pose: np.ndarray,
        twist: np.ndarray,
        visible: bool,
    ):
        self.id = id
        self.pose = pose
        self.twist = twist
        self.visible = visible


class PartialWorldState:
    """
    A class that contains all the ball, and robot states
    """

    __slots = ["our_robots", "their_robots", "ball"]

    our_robots: List[RobotState]
    their_robots: List[RobotState]
    ball: rc.Ball

    def __init__(
        self,
        our_robots: List[RobotState],
        their_robots: List[RobotState],
        ball: rc.Ball,
    ):
        self.our_robots = our_robots
        self.their_robots = their_robots
        self.ball = ball


def robotstate_to_partial_robot(robot_msg: msg.RobotState, index: int) -> RobotState:
    """
    :return: robot state class representing the state of the robot, partially representing the larger Robot class.
    """
    robot_id = index

    x = robot_msg.pose.position.x
    y = robot_msg.pose.position.y
    theta = robot_msg.pose.heading
    pose = np.array([x, y, theta])

    dx = robot_msg.velocity.linear.x
    dy = robot_msg.velocity.linear.y
    dtheta = robot_msg.velocity.angular
    twist = np.array([dx, dy, dtheta])

    is_visible = robot_msg.visible

    robot = RobotState(robot_id, pose, twist, is_visible)

    return robot


def robotstatus_to_partial_robot(robot_msg: msg.RobotStatus) -> RobotStatus:
    """
    :return: robot status class representing the status of the robot, partially representing the larger Robot class.
    """
    robot_id = robot_msg.robot_id

    ball_sense = robot_msg.has_ball_sense

    kicker_charged = robot_msg.kicker_charged

    kicker_healthy = robot_msg.kicker_healthy

    lethal_fault = robot_msg.fpga_error

    robot = RobotStatus(
        robot_id, ball_sense, kicker_charged, kicker_healthy, lethal_fault
    )

    return robot


def ballstate_to_ball(ball_msg: msg.BallState) -> rc.Ball:
    """
    :return: ball class representing the state of the ball.
    """
    x = ball_msg.position.x
    y = ball_msg.position.y
    pos = np.array([x, y])

    dx = ball_msg.velocity.x
    dy = ball_msg.velocity.y
    vel = np.array([dx, dy])

    visible = ball_msg.visible
    ball = rc.Ball(pos, vel, visible)

    return ball


def build_game_info(
    play_state_msg: msg.PlayState, match_state_msg: msg.MatchState
) -> rc.GameInfo:
    """
    :return: GameInfo class from rc.py
    """

    period = rc.GamePeriod(match_state_msg.period)

    state = rc.GameState(play_state_msg.state)

    restart = rc.GameRestart(play_state_msg.restart)

    our_restart = play_state_msg.our_restart

    game_info = rc.GameInfo(
        period,
        state,
        restart,
        our_restart,
        np.array(
            [
                play_state_msg.placement_point.x,
                play_state_msg.placement_point.y,
            ]
        ),
    )

    return game_info


def field_msg_to_field(field_msg: msg.FieldDimensions) -> rc.Field:
    """
    :return: Field class from rc.py
    """

    length = field_msg.length
    width = field_msg.width
    border = field_msg.border
    line_width = field_msg.line_width
    goal_width = field_msg.goal_width
    goal_depth = field_msg.goal_depth
    goal_height = field_msg.goal_height
    penalty_short_dist = field_msg.penalty_short_dist
    penalty_long_dist = field_msg.penalty_long_dist
    center_radius = field_msg.center_radius
    center_diameter = field_msg.center_diameter
    goal_flat = field_msg.goal_flat
    floor_length = field_msg.floor_length
    floor_width = field_msg.floor_width

    field = rc.Field(
        length,
        width,
        border,
        line_width,
        goal_width,
        goal_depth,
        goal_height,
        penalty_short_dist,
        penalty_long_dist,
        center_radius,
        center_diameter,
        goal_flat,
        floor_length,
        floor_width,
    )

    return field


def worldstate_message_converter(msg: msg.WorldState) -> PartialWorldState:
    """
    :return: partial world state class representing the state of the robots and ball.
    """
    our_robots: List[RobotState]
    our_robots = []

    their_robots: List[RobotState]
    their_robots = []

    for i in range(len(msg.our_robots)):
        our_robots.append(robotstate_to_partial_robot(msg.our_robots[i], i))

    for i in range(len(msg.their_robots)):
        their_robots.append(robotstate_to_partial_robot(msg.their_robots[i], i))

    ball = ballstate_to_ball(msg.ball)

    world_state = PartialWorldState(our_robots, their_robots, ball)

    return world_state


def robot_creator(
    robot_state: RobotState, robot_status: RobotStatus = None
) -> rc.Robot:
    """
    A function which combines the robot state and robot status to create a rc.Robot class
        :return: Robot class from rc.Robot representing the status and state of the robot
    """

    if robot_status is None:
        is_ours = False
        robot_id = robot_state.id
        ball_sense = False
        kicker_charged = False
        kicker_healthy = False
        lethal_fault = False
    else:
        is_ours = True
        robot_id = robot_state.id
        ball_sense = robot_status.has_ball_sense
        kicker_charged = robot_status.kicker_charged
        kicker_healthy = robot_status.kicker_healthy
        lethal_fault = robot_status.lethal_fault
    pose = robot_state.pose
    twist = robot_state.twist
    is_visible = robot_state.visible

    robot = rc.Robot(
        robot_id,
        is_ours,
        pose,
        twist,
        is_visible,
        ball_sense,
        kicker_charged,
        kicker_healthy,
        lethal_fault,
    )

    return robot


def worldstate_creator(
    partial_world_state: PartialWorldState,
    robot_statuses: List[RobotStatus],
    game_info: rc.GameInfo,
    field: rc.Field,
    goalie_id: int,
) -> rc.WorldState:
    """
    A function which combines the partial world state, robot statuses, game info, and field to create a whole world state
        :return: a world state as a rc.WorldState object
    """

    our_partial_robots = partial_world_state.our_robots
    their_partial_robots = partial_world_state.their_robots
    our_robots = []
    their_robots = []

    for i in range(len(our_partial_robots)):
        robot = robot_creator(our_partial_robots[i], robot_statuses[i])
        our_robots.append(robot)

    for partial_bot in their_partial_robots:
        robot = robot_creator(partial_bot, None)
        their_robots.append(robot)

    if game_info is None:
        game_info = None

    world_state = rc.WorldState(
        our_robots,
        their_robots,
        partial_world_state.ball,
        game_info,
        field,
        goalie_id,
    )

    return world_state
