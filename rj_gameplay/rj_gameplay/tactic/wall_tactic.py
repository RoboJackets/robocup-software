"""Tactic to build a wall between mark pt (e.g. ball) and defense pt (e.g. goal)."""

from dataclasses import dataclass
from typing import List, Optional, Any
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar, Any

import stp.action as action
import stp.rc as rc
import stp.tactic as tactic
import stp.role as role

from rj_gameplay.skill import move
import rj_gameplay.eval
import rj_gameplay.skill as skills

from rj_msgs.msg import PathTargetMotionCommand

import stp.skill as skill
import numpy as np

# TODO: replace w/ global param server
from stp.utils.constants import RobotConstants, BallConstants
import stp.global_parameters as global_parameters

MIN_WALL_RAD = None


class wall_cost(role.CostFn):
    """Cost function for role request."""

    def __init__(self, wall_pt: np.ndarray = None, scale: float = 1.0):

        self.wall_pt = wall_pt
        self.scale = scale

    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional[role.RoleResult],
        world_state: rc.WorldState,
    ) -> float:

        if robot is None:
            return 9999

        wall_pt = np.array([0.0, 0.0]) if self.wall_pt is None else self.wall_pt

        # TODO(#1669): Remove this once role assignment no longer assigns non-visible robots
        if not robot.visible:
            return 9999  # float('inf') threw ValueError

        # TODO: fix goalie assignment issue the right way
        # if np.linalg.norm(robot.pose[0:2] - world_state.field.our_goal_loc) < MIN_WALL_RAD:
        #     return 9999
        switch_cost = 0
        if prev_result and prev_result.is_filled():
            switch_cost = 1 * (prev_result.role.robot.id != robot.id)

        # costs should be in seconds, not dist
        return (
            self.scale
            * np.linalg.norm(robot.pose[0:2] - wall_pt)
            / global_parameters.soccer.robot.max_speed
            + switch_cost
        )

    def unassigned_cost_fn(
        self,
        prev_result: Optional[role.RoleResult],
        world_state: rc.WorldState,
    ) -> float:

        # TODO: Implement real unassigned cost function
        return role.BIG_STUPID_NUMBER_CONST_FOR_UNASSIGNED_COST_PLS_CHANGE

    # def switch_cost_fn(
    #     self,
    #     prev_result: Optional["RoleResult"],
    #     world_state: rc.WorldState,
    #     sticky_weight: float
    # ) -> float:

    #     return


class WallTactic(tactic.ITactic):
    def __init__(
        self,
        action_client_dict: Dict[Type[Any], List[Any]],
        priority=role.Priority.MEDIUM,
        cost_scale: float = 1.0,
    ):

        self._action_client_dict = action_client_dict

        # create move SkillEntry for every robot
        self.move_var = tactic.SkillEntry(move.Move(action_client_dict))

        # create empty cost_var (filled in get_requests)
        self.cost_var = wall_cost(scale=cost_scale)
        self.priority = priority

    def compute_props(self):
        pass

    def create_request(self, **kwargs) -> role.RoleRequest:
        """Creates a sane default RoleRequest.
        :return: A list of size 1 of a sane default RoleRequest.
        """
        pass

    def get_requests(
        self, world_state: rc.WorldState, wall_pt, props
    ) -> List[tactic.RoleRequests]:
        """
        :return: A list of role requests for move skills needed
        """
        if world_state and world_state.ball.visible:
            self.move_var.skill.target_point = wall_pt
            self.move_var.skill.face_point = world_state.ball.pos
            robot = self.move_var.skill.robot
            self.cost_var.wall_pt = wall_pt

        # create RoleRequest for each SkillEntry
        role_requests = {
            self.move_var: [role.RoleRequest(self.priority, False, self.cost_var)]
            for _ in range(1)
        }

        return role_requests

    def tick(
        self, world_state: rc.WorldState, role_results: tactic.RoleResults
    ) -> List[tactic.SkillEntry]:
        """
        :return: A list of skills depending on which roles are filled
        """

        # create list of skills based on if RoleResult exists for SkillEntry
        skills = [self.move_var if role_results[self.move_var] else None]

        return skills

    def is_done(self, world_state):
        """
        :return boolean indicating if tactic is done
        """
        if not self.move_var.skill.is_done(world_state):
            return False
        return True
