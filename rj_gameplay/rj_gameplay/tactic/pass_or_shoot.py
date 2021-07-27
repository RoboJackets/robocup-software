from dataclasses import dataclass
from typing import List, Optional, Callable
import stp.rc as rc
import stp.tactic as tactic
import stp.role as role
from rj_gameplay.skill import shoot, capture, pivot_kick, line_kick
from rj_gameplay.tactic import striker_tactic, pass_tactic
import stp.skill as skill
import numpy as np
from math import atan2

OPPONENT_SPEED = 1.5
KICK_SPEED = 6.0
EFF_BLOCK_WIDTH = 0.7

def shoot_cost(world_state: rc.WorldState) -> bool:
    for bot in world_state.our_robots:
        if bot.has_ball_sense:
            robot = bot
    for bot in world_state.our_robots:
        if bot.id != robot.id and np.linalg.norm(bot.pose[0:2] - robot.pose[0:2]) < 2.0:
            return False
    return True

class PassOrShoot(tactic.ITactic):
    """
	A striker tactic which receives then shoots the ball
	"""

    def __init__(self, target_point: np.ndarray, shoot_cost: Callable[[rc.WorldState, rc.Robot], bool]) -> None:
        self.target_point: np.ndarray = np.array([0., 9.])
        self.cost = pass_or_shoot_cost
        self.shoot = None
        self.capture = tactic.SkillEntry(capture.Capture())
        self.capture_cost = striker_tactic.CaptureCost()
        self.striker_tactic = striker_tactic.LineKickStrikerTactic(target_point=self.target_point)
        self.pass_tactic = pass_tactic.Pass(self.target_point, self.capture_cost ,pass_tactic.PassToOpenReceiver())

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

        role_requests: tactic.RoleRequests = {}

        if not self.capture.skill.is_done(world_state):
            capture_request = role.RoleRequest(role.Priority.MEDIUM, True,
                                           self.capture_cost)
            role_requests[self.capture] = [capture_request]
        elif self.capture.skill.is_done(world_state) and self.shoot:
            role_requests = self.striker_tactic.get_requests(world_state, props)
        elif self.capture.skill.is_done(world_state) and not self.shoot:
            role_requests = self.pass_tactic.get_requests(world_state, props)

        return role_requests

    def tick(self, role_results: tactic.RoleResults,
             world_state: rc.WorldState) -> List[tactic.SkillEntry]:
        """
		:return: list of skills
		"""
        if not self.capture.skill.is_done(world_state):
            capture_result = role_results[self.capture]
                if capture_result and capture_result[0].is_filled():
                    return [self.capture]
        if self.capture.skill.is_done(world_state):
            if self.shoot is None:
                self.shoot = shoot_cost(world_state)
        if shoot_result and shoot_result[0].is_filled():
            self.shoot.skill.target_point = find_target_point(world_state, kick_speed=KICK_SPEED)
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

        striker_request = role.RoleRequest(role.Priority.MEDIUM, True,
                                           self.capture_cost)
        role_requests: tactic.RoleRequests = {}

        role_requests[self.shoot] = [striker_request]

        return role_requests

    def tick(self, role_results: tactic.RoleResults,
             world_state: rc.WorldState) -> List[tactic.SkillEntry]:
        """
		:return: list of skills
		"""

        shoot_result = role_results[self.shoot]

        if shoot_result and shoot_result[0].is_filled():
            self.shoot.skill.target_point = find_target_point(world_state, kick_speed=KICK_SPEED)
            return [self.shoot]

        return []

    def is_done(self, world_state) -> bool:
        return self.shoot.skill.is_done(world_state)
