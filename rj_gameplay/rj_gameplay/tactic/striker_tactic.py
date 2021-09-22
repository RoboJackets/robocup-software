from dataclasses import dataclass
from typing import List, Optional
import stp.rc as rc
import stp.tactic as tactic
import stp.role as role
from rj_gameplay.skill import shoot, capture, pivot_kick, line_kick
import stp.skill as skill
import numpy as np
from math import atan2

OPPONENT_SPEED = 1.5
KICK_SPEED = 4.5
EFF_BLOCK_WIDTH = 0.7


def blocker_margin(kick_origin: np.array, kick_target: np.array,
                   kick_speed: float, blocker: rc.Robot):
    if not blocker.visible:
        return np.inf

    kick_vector = kick_target - kick_origin
    kick_dist = np.linalg.norm(kick_vector)
    kick_vector /= kick_dist
    kick_perp = np.array([kick_vector[1], -kick_vector[0]])

    blocker_position = blocker.pose[0:2]

    # Calculate blocker intercept
    blocker_intercept_dist_along_kick = np.dot(blocker_position - kick_origin,
                                               kick_vector)
    blocker_intercept_dist_along_kick = np.clip(
        blocker_intercept_dist_along_kick, a_min=0, a_max=kick_dist)
    blocker_intercept = kick_origin + kick_vector * blocker_intercept_dist_along_kick

    blocker_distance = np.clip(
        np.linalg.norm(blocker_intercept - blocker_position) - EFF_BLOCK_WIDTH,
        a_min=0.0,
        a_max=np.inf)

    blocker_time = np.abs(blocker_distance) / OPPONENT_SPEED

    # Doesn't include friction...oops?
    ball_time = np.linalg.norm(blocker_intercept - kick_origin) / kick_speed

    return blocker_time - ball_time


def kick_cost(point: np.array, kick_speed: float, kick_origin: np.array,
              world_state: rc.WorldState):
    margins = [
        blocker_margin(kick_origin, point, kick_speed, blocker)
        for blocker in world_state.their_robots
    ]
    return -min(margins)


def find_target_point(world_state: rc.WorldState, kick_speed) -> np.ndarray:
    goal_y = world_state.field.length_m
    cost = 0

    ball_pos = world_state.ball.pos

    # Heuristic: limit where we kick if we're very wide
    xmin = -0.43
    xmax = 0.43
    if abs(ball_pos[0]) > 1:
        kick_extent = -1 / ball_pos[0]
        if kick_extent < 0:
            xmin = np.clip(kick_extent, a_min=xmin, a_max=0)
        elif kick_extent > 0:
            xmax = np.clip(kick_extent, a_min=0, a_max=xmax)

    try_points = [
        np.array([x, goal_y]) for x in np.arange(xmin, xmax, step=0.05)
    ]

    cost, point = min([(kick_cost(point, kick_speed, world_state.ball.pos,
                                  world_state), point)
                       for point in try_points],
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

            goal_y = world_state.field.length_m
            goal_pos = np.array([0., goal_y])
            robot_to_ball = ball_pos - robot_pos
            # robot_to_ball /= np.linalg.norm(robot_to_ball) + 1e-6
            ball_to_goal = goal_pos - ball_pos
            ball_to_goal /= np.linalg.norm(ball_to_goal) + 1e-6

            LINE_KICK_APPROACH_DISTANCE = 0.2
            target_pos = ball_pos - ball_to_goal * LINE_KICK_APPROACH_DISTANCE
            dist_to_ball = np.linalg.norm(target_pos - robot_pos)

            switch_cost = 0.0
            if prev_result is not None and prev_result.is_filled():
                switch_cost += 1.0 * (prev_result.role.robot.id != robot.id)

            return 10 * dist_to_ball + 0.5 * switch_cost


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
            pivot_kick.PivotKick(robot=None,
                                 chip=False,
                                 kick_speed=KICK_SPEED,
                                 target_point=target_point,
                                 threshold=0.05))

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

        # TODO change priority float to something useful
        striker_request = role.RoleRequest(2.0, True,
                                           self.capture_cost)
        role_requests: tactic.RoleRequests = {}

        striker = [
            robot for robot in world_state.our_robots if robot.has_ball_sense
        ]

        if striker:
            role_requests[self.capture] = []
            role_requests[self.shoot] = [striker_request]
        else:
            role_requests[self.capture] = [striker_request]
            role_requests[self.shoot] = []

        return role_requests

    def tick(
        self,
        world_state: rc.WorldState,
        role_results: tactic.RoleResults,
    ) -> List[tactic.SkillEntry]:
        """
		:return: list of skills
		"""

        capture_result = role_results[self.capture]
        shoot_result = role_results[self.shoot]

        if capture_result and capture_result[0].is_filled():
            return [self.capture]
        if shoot_result and shoot_result[0].is_filled():
            self.shoot.skill.target_point = find_target_point(
                world_state, kick_speed=KICK_SPEED)
            return [self.shoot]

        return []

    def is_done(self, world_state) -> bool:
        return self.shoot.skill.is_done(world_state)


class LineKickStrikerTactic(tactic.ITactic):
    """
	A striker tactic which receives then shoots the ball
	"""
    def __init__(self, target_point: np.ndarray, cost: role.CostFn = None):
        self.cost = cost  # unused
        self.target_point = target_point
        self.shoot = tactic.SkillEntry(
            line_kick.LineKickSkill(robot=None, target_point=None))
        self.capture_cost = CaptureCost()

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

        # TODO change priority float to something useful
        striker_request = role.RoleRequest(2.0, True,
                                           self.capture_cost)
        role_requests: tactic.RoleRequests = {}

        role_requests[self.shoot] = [striker_request]

        return role_requests

    def tick(self, world_state: rc.WorldState,
             role_results: tactic.RoleResults) -> List[tactic.SkillEntry]:
        """
		:return: list of skills
		"""

        shoot_result = role_results[self.shoot]

        if shoot_result and shoot_result[0].is_filled():
            self.shoot.skill.target_point = find_target_point(
                world_state, kick_speed=KICK_SPEED)
            shooter_vel = shoot_result[0].role.robot.twist[:2]
            if world_state is not None and world_state.game_info.is_penalty():
                dist_to_goal = world_state.field.their_goal_loc[
                    1] - world_state.ball.pos[1]
                if dist_to_goal > 4.0:
                    self.shoot.skill.kick_speed = max(
                        0.0, 2.0 - np.linalg.norm(shooter_vel))
                elif dist_to_goal > 3.5:
                    self.shoot.skill.kick_speed = max(
                        0.0, 1.5 - np.linalg.norm(shooter_vel))
            return [self.shoot]

        return []

    def is_done(self, world_state) -> bool:
        return self.shoot.skill.is_done(world_state)
