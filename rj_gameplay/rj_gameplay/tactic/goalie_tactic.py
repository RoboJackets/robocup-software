"""Tactic to build a wall between mark pt (e.g. ball) and defense pt (e.g. goal)."""

from dataclasses import dataclass
from typing import List, Optional
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar

import stp.action as action
import stp.rc as rc
import stp.tactic as tactic
import stp.role as role

import rj_gameplay.eval
import rj_gameplay.skill as skills
from rj_gameplay.skill import move
import stp.skill as skill
import numpy as np
# TODO: replace w/ global param server
from stp.utils.constants import RobotConstants, BallConstants
import stp.global_parameters as global_parameters


class goalie_cost(role.CostFn):
    """Cost function for role request. Want only the goalie to be selected.
    """
    def __call__(self, robot: rc.Robot, prev_result: Optional["RoleResult"],
                 world_state: rc.WorldState) -> float:

        if world_state.game_info is not None:
            if robot.id == world_state.game_info.goalie_id:
                return -1.0
        return 999.0

def get_goalie_pt(world_state: rc.WorldState) -> np.ndarray:
    """Finds point for goalie to best intercept a shot.
    :return numpy point
    """
    # TODO: param server this const
    # TODO: param server any constant from stp/utils/constants.py (this includes BallConstants)
    ball_pt = world_state.ball.pos
    goal_pt = world_state.field.our_goal_loc

    PCT_TO_BALL = 0.50

    # get direction vec to ball
    dir_vec = (ball_pt - goal_pt) / np.linalg.norm(ball_pt - goal_pt)
    wall_vec = np.array([dir_vec[1], -dir_vec[0]])

    # find and return pt
    mid_pt = goal_pt + (dir_vec * PCT_TO_BALL * np.linalg.norm(ball_pt - goal_pt))
    return mid_pt


class GoalieTactic(tactic.ITactic):
    def __init__(self):

        # create move SkillEntry
        self.move_se = tactic.SkillEntry(move.Move())
        # TODO: intercept skill

        # TODO: rename cost_list to role_cost in other gameplay files
        self.role_cost = goalie_cost()

    def compute_props(self):
        pass

    def create_request(self, **kwargs) -> role.RoleRequest:
        """Creates a sane default RoleRequest.
        :return: A list of size 1 of a sane default RoleRequest.
        """
        pass

    def get_requests(self, world_state: rc.WorldState,
                     props) -> List[tactic.RoleRequests]:
        """
        :return: A list of role requests for move skills needed
        """

        if world_state and world_state.ball.visible:
            # update move skill
            self.move_se.skill.target_point = get_goalie_pt(world_state)
            self.move_se.skill.face_point = world_state.ball.pos

        # create RoleRequest for each SkillEntry
        role_requests = {}
        role_requests[self.move_se] = [role.RoleRequest(role.Priority.HIGH, False, self.role_cost)]
        return role_requests

    def tick(self,
             role_results: tactic.RoleResults) -> List[tactic.SkillEntry]:
        """
        :return: A list of skills depending on which roles are filled
        """

        # create list of skills based on if RoleResult exists for SkillEntry
        skills = []
        if role_results[self.move_se][0]:
            skills.append(self.move_se)

        return skills

    def is_done(self, world_state):
        """
        :return boolean indicating if tactic is done
        """
        return self.move_se.skill.is_done(world_state)
