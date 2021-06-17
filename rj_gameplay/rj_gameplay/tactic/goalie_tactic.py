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
from rj_gameplay.skill import move, intercept
import stp.skill as skill
import numpy as np
# TODO: replace w/ global param server
from stp.utils.constants import RobotConstants, BallConstants
import stp.global_parameters as global_parameters
from stp.local_parameters import Param


class goalie_cost(role.CostFn):
    """Cost function for role request. Want only the designated goalie to be selected.
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
    # TODO: param server any constant from stp/utils/constants.py (this includes BallConstants)
    ball_pt = world_state.ball.pos
    goal_pt = world_state.field.our_goal_loc

    # TODO: param server this const
    PCT_TO_BALL = 0.10

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

        self.intercept_se = tactic.SkillEntry(intercept.Intercept())

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

        # TODO: this const is copy-pasted from wall_tactic 
        # put into common param file: https://www.geeksforgeeks.org/global-keyword-in-python/

        # dist is slightly greater than penalty box bounds
        box_w = world_state.field.penalty_long_dist_m
        box_h = world_state.field.penalty_short_dist_m
        line_w = world_state.field.line_width_m
        MIN_WALL_RAD = RobotConstants.RADIUS + line_w + np.hypot(box_w / 2, box_h)

        role_requests = {}
        if world_state and world_state.ball.visible:
            ball_to_goal_dist = np.linalg.norm(world_state.field.our_goal_loc - world_state.ball.pos)
            if ball_to_goal_dist < MIN_WALL_RAD:
                print("INTERCEPT"*80)
                # intercept when inside wall

                # update intercept skill
                self.intercept_se.skill.target_point = world_state.ball.pos

                # create RoleRequest for each SkillEntry
                role_requests[self.intercept_se] = [role.RoleRequest(role.Priority.HIGH, False, self.role_cost)]
            else:
                # else, track ball normally

                # update move skill
                self.move_se.skill.target_point = get_goalie_pt(world_state)
                self.move_se.skill.face_point = world_state.ball.pos

                # create RoleRequest for each SkillEntry
                role_requests[self.move_se] = [role.RoleRequest(role.Priority.HIGH, False, self.role_cost)]
        return role_requests

    def tick(self,
             role_results: tactic.RoleResults) -> List[tactic.SkillEntry]:
        """
        :return: A list of skills depending on which roles are filled
        """

        # create list of skills based on if RoleResult exists for SkillEntry
        skills = []
        if role_results[self.move_se]:
            if role_results[self.move_se][0]:
                skills.append(self.move_se)
        elif role_results[self.intercept_se]:
            if role_results[self.intercept_se][0]:
                skills.append(self.intercept_se)

        return skills

    def is_done(self, world_state):
        """
        :return boolean indicating if tactic is done
        """
        return self.move_se.skill.is_done(world_state)
