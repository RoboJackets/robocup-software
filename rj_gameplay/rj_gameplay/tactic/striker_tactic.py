from dataclasses import dataclass
from typing import List, Tuple
import stp.rc as rc
import stp.tactic as tactic
import stp.role as role
from rj_gameplay.skill import capture, pivot_kick, line_kick
import stp.skill as skill
import numpy as np
from math import atan2

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

        # for role in self.assigned_roles:
        #     if not role.is_done(world_state):
        #         print("Striker: ", role.robot.id)

        # if low performance, make this not a for loop since it's only one tactic
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
            print("Striker: ", robot.id)

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
