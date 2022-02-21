from dataclasses import dataclass
from typing import List, Optional, Tuple
import stp.rc as rc
import stp.tactic as tactic
import stp.role as role
from rj_gameplay.skill import capture, pivot_kick, line_kick
import stp.skill as skill
import numpy as np
from math import atan2

# TODO: move all of this logic to StrikerRole
OPPONENT_SPEED = 1.5
KICK_SPEED = 4.5
EFF_BLOCK_WIDTH = 0.7


def blocker_margin(
    kick_origin: np.array,
    kick_target: np.array,
    kick_speed: float,
    blocker: rc.Robot,
):
    if not blocker.visible:
        return np.inf

    kick_vector = kick_target - kick_origin
    kick_dist = np.linalg.norm(kick_vector)
    kick_vector /= kick_dist
    kick_perp = np.array([kick_vector[1], -kick_vector[0]])

    blocker_position = blocker.pose[0:2]

    # Calculate blocker intercept
    blocker_intercept_dist_along_kick = np.dot(
        blocker_position - kick_origin, kick_vector
    )
    blocker_intercept_dist_along_kick = np.clip(
        blocker_intercept_dist_along_kick, a_min=0, a_max=kick_dist
    )
    blocker_intercept = kick_origin + kick_vector * blocker_intercept_dist_along_kick

    blocker_distance = np.clip(
        np.linalg.norm(blocker_intercept - blocker_position) - EFF_BLOCK_WIDTH,
        a_min=0.0,
        a_max=np.inf,
    )

    blocker_time = np.abs(blocker_distance) / OPPONENT_SPEED

    # Doesn't include friction...oops?
    ball_time = np.linalg.norm(blocker_intercept - kick_origin) / kick_speed

    return blocker_time - ball_time


def kick_cost(
    point: np.array,
    kick_speed: float,
    kick_origin: np.array,
    world_state: rc.WorldState,
):
    margins = [
        blocker_margin(kick_origin, point, kick_speed, blocker)
        for blocker in world_state.their_robots
    ]
    return -min(margins)


# TODO: move imports to top once this file fixed
from rj_gameplay.role import striker
import stp

from rj_msgs.msg import RobotIntent


class StrikerTactic(stp.tactic.Tactic):
    """
    Captures ball and shoots. This Tactic merely holds the StrikerRole and handles its assignment for the Play level.
    """

    def __init__(self, world_state: stp.rc.WorldState):
        super().__init__(world_state)

        self._role_requests.append(
            (
                stp.role.cost.PickClosestToPoint(world_state.ball.pos),
                striker.StrikerRole,
            )
        )

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[Tuple[int, RobotIntent]]:
        # returns list of (robot_id, robot_intent)

        # assumes all roles requested are filled, because tactic is one unit
        if len(self.assigned_roles) != len(self._role_requests):
            self.init_roles(world_state)

        return [(role.robot.id, role.tick(world_state)) for role in self.assigned_roles]

    def is_done(
        self,
        world_state: stp.rc.WorldState,
    ) -> bool:
        return all([role.is_done(world_state) for role in self.assigned_roles])

    def init_roles(
        self,
        world_state: stp.rc.WorldState,
    ):
        robot = self.assigned_robots[0]
        role = self._role_requests[0][1]
        if role is striker.StrikerRole:
            self.assigned_roles.append(role(robot))

    @property
    def needs_assign(self):
        # never needs assign after init
        # TODO: make this + pass tac part of the superclass
        return False


class LineKickStrikerTactic(tactic.ITactic):
    """
    A striker tactic which receives then shoots the ball
    """

    def __init__(self, target_point: np.ndarray, cost: role.CostFn = None):
        self.cost = cost  # unused
        self.target_point = target_point
        self.shoot = tactic.SkillEntry(
            line_kick.LineKickSkill(robot=None, target_point=None)
        )
        self.capture_cost = CaptureCost()

    def compute_props(self):
        pass

    def create_request(self, **kwargs) -> role.RoleRequest:
        """
        Creates a sane default RoleRequest.
        :return: A list of size 1 of a sane default RoleRequest.
        """
        pass

    def get_requests(
        self, world_state: rc.WorldState, props
    ) -> List[tactic.RoleRequests]:

        striker_request = role.RoleRequest(
            role.Priority.MEDIUM, True, self.capture_cost
        )
        role_requests: tactic.RoleRequests = {}

        role_requests[self.shoot] = [striker_request]

        return role_requests

    def tick(
        self, world_state: rc.WorldState, role_results: tactic.RoleResults
    ) -> List[tactic.SkillEntry]:
        """
        :return: list of skills
        """

        shoot_result = role_results[self.shoot]

        if shoot_result and shoot_result[0].is_filled():
            self.shoot.skill.target_point = find_target_point(
                world_state, kick_speed=KICK_SPEED
            )
            shooter_vel = shoot_result[0].role.robot.twist[:2]
            if world_state is not None and world_state.game_info.is_penalty():
                dist_to_goal = (
                    world_state.field.their_goal_loc[1] - world_state.ball.pos[1]
                )
                if dist_to_goal > 4.0:
                    self.shoot.skill.kick_speed = max(
                        0.0, 2.0 - np.linalg.norm(shooter_vel)
                    )
                elif dist_to_goal > 3.5:
                    self.shoot.skill.kick_speed = max(
                        0.0, 1.5 - np.linalg.norm(shooter_vel)
                    )
            return [self.shoot]

        return []

    def is_done(self, world_state) -> bool:
        return self.shoot.skill.is_done(world_state)
