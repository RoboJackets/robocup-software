from dataclasses import dataclass
from typing import List, Optional
import stp.rc as rc
import stp.tactic as tactic
import stp.role as role
from rj_gameplay.skill import shoot, capture, pivot_kick
import stp.skill as skill
import numpy as np
from math import atan2

OPPONENT_SPEED = 1.5
KICK_SPEED = 7.0
EFF_BLOCK_WIDTH = 0.7


def blocker_margin(kick_origin: np.array, kick_target: np.array, kick_speed: float, blocker: rc.Robot):
    if not blocker.visible:
        return np.inf

    kick_vector = kick_target - kick_origin
    kick_dist = np.linalg.norm(kick_vector)
    kick_vector /= kick_dist
    kick_perp = np.array([kick_vector[1], -kick_vector[0]])

    blocker_position = blocker.pose[0:2]

    # Calculate blocker intercept
    blocker_intercept_dist_along_kick = np.dot(blocker_position - kick_origin, kick_vector)
    blocker_intercept_dist_along_kick = np.clip(blocker_intercept_dist_along_kick, a_min=0, a_max=kick_dist)
    blocker_intercept = kick_origin + kick_vector * blocker_intercept_dist_along_kick

    blocker_distance = np.clip(np.linalg.norm(blocker_intercept - blocker_position) - EFF_BLOCK_WIDTH, a_min=0.0,
                               a_max=np.inf)

    print(f"Target {kick_target}, blocker {blocker.id} distance {blocker_distance}")

    blocker_time = np.abs(blocker_distance) / OPPONENT_SPEED

    # Doesn't include friction...oops?
    ball_time = np.linalg.norm(blocker_intercept - kick_origin) / kick_speed

    return blocker_time - ball_time


def kick_cost(point: np.array, kick_speed: float, kick_origin: np.array, world_state: rc.WorldState):
    margins = [
        blocker_margin(kick_origin, point, kick_speed, blocker)
        for blocker in world_state.their_robots
    ]
    return -min(margins)


def find_target_point(world_state: rc.WorldState, kick_speed) -> np.ndarray:
    goal_y = world_state.field.length_m
    cost = 0
    try_points = [
        np.array([x, goal_y])
        for x in np.arange(-0.45, 0.45, 0.05)
    ]

    kicker = [
        robot for robot in world_state.our_robots if robot.has_ball_sense
    ]
    if kicker:
        kicker = kicker[0]
    else:
        return None

    cost, point = min(
        [(kick_cost(point, kick_speed, world_state.ball.pos, world_state), point) for point in try_points],
        key=lambda x: x[0])

    return point


class CaptureCost(role.CostFn):
    """
    A cost function for how to choose a robot that will pass
    """

    def __call__(self, robot: rc.Robot, prev_result: Optional["RoleResult"],
                 world_state: rc.WorldState) -> float:
        if robot.has_ball_sense:
            return 0
        else:
            robot_pos = robot.pose[0:2]
            ball_pos = world_state.ball.pos[0:2]
            dist_to_ball = np.linalg.norm(ball_pos - robot_pos)
            return dist_to_ball


class StrikerTactic(tactic.ITactic):
    """
	A striker tactic which receives then shoots the ball
	"""

    def __init__(self, target_point: np.ndarray, cost: role.CostFn = None):
        self.cost = cost  # unused
        self.target_point = target_point
        self.capture = tactic.SkillEntry(capture.Capture())
        self.capture_cost = CaptureCost()
        self.shoot = tactic.SkillEntry(
            pivot_kick.PivotKick(robot=None, chip=False, kick_speed=40., target_point=target_point, threshold=0.05))

    def compute_props(self):
        pass

    def create_request(self, **kwargs) -> role.RoleRequest:
        """
		Creates a sane default RoleRequest.
		:return: A list of size 1 of a sane default RoleRequest.
		"""
        pass

    def get_requests(self, world_state: rc.WorldState,
                     props) -> List[tactic.RoleRequests]:

        striker_request = role.RoleRequest(role.Priority.HIGH, True,
                                           self.capture_cost)
        role_requests: tactic.RoleRequests = {}

        if self.shoot.skill.is_done(world_state):
            self.shoot = tactic.SkillEntry(
                shoot.Shoot(chip=False, kick_speed=8., target_point=self.target_point))

        striker = [robot for robot in world_state.our_robots if robot.has_ball_sense]

        if striker:
            role_requests[self.capture] = []
            role_requests[self.shoot] = [striker_request]
        else:
            role_requests[self.capture] = [striker_request]
            role_requests[self.shoot] = []

        return role_requests

    def tick(self, role_results: tactic.RoleResults,
             world_state: rc.WorldState) -> List[tactic.SkillEntry]:
        """
		:return: list of skills
		"""

        capture_result = role_results[self.capture]
        shoot_result = role_results[self.shoot]

        if capture_result and capture_result[0].is_filled():
            return [self.capture]
        if shoot_result and shoot_result[0].is_filled():
            self.shoot.skill.target_point = find_target_point(world_state, kick_speed=KICK_SPEED)
            return [self.shoot]

        return []

    def is_done(self, world_state) -> bool:
        return self.shoot.skill.is_done(world_state)
