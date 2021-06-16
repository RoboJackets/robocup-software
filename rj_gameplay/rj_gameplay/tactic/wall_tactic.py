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

class wall_cost(role.CostFn):
    """Cost function for role request.
    """
    def __init__(self, wall_pt: np.ndarray=None):
        self.wall_pt = wall_pt

    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState
    ) -> float:

        if robot is None or self.wall_pt is None:
            return 0
        return np.linalg.norm(robot.pose[0:2]-self.wall_pt)

def find_wall_pts(num_wallers: int, world_state: rc.WorldState) -> List[np.ndarray]:
    """Calculates num_wallers points to form a wall between the ball and goal.
    :return list of wall_pts (as numpy arrays)
    """
    # TODO: param server this const
    # TODO: param server any constant from stp/utils/constants.py (this includes BallConstants)
    ball_pt = world_state.ball.pos
    goal_pt = world_state.field.our_goal_loc

    WALL_SPACING = BallConstants.RADIUS

    # dist is slightly greater than penalty box bounds
    box_w = world_state.field.penalty_long_dist_m
    box_h = world_state.field.penalty_short_dist_m
    line_w = world_state.field.line_width_m
    DIST_FROM_DEF = RobotConstants.RADIUS + line_w + np.hypot(box_w/2, box_h)

    # check if vision is up and running
    # (if it is these points should not be equal)
    # if not world_state.ball.visible:
    #    return None

    # get direction vec
    dir_vec = (ball_pt - goal_pt) / np.linalg.norm(ball_pt - goal_pt)
    wall_vec = np.array([dir_vec[1], -dir_vec[0]])

    # find mid_pt
    mid_pt = goal_pt + (dir_vec * DIST_FROM_DEF)
    wall_pts = [mid_pt]

    # set wall points in middle out pattern, given wall dir vector and WALL_SPACING constant
    wall_pts = [mid_pt]
    for i in range(num_wallers-1):
        mult = i//2 + 1
        delta = (mult * (2 * RobotConstants.RADIUS + WALL_SPACING)) * wall_vec 
        if i % 2: delta = -delta
        wall_pts.append(mid_pt + delta)

    return wall_pts

class WallTactic(tactic.ITactic):

    def __init__(self, num_wallers: int):

        self.num_wallers = num_wallers

        # create move SkillEntry for every robot
        self.move_list = [
            tactic.SkillEntry(move.Move()) 
            for _ in range(num_wallers)
        ]

        # create empty cost_list (filled in get_requests)
        self.cost_list = [
            wall_cost()
            for _ in range(self.num_wallers)
        ]
        
    def compute_props(self):
        pass

    def create_request(self, **kwargs) -> role.RoleRequest:
        """Creates a sane default RoleRequest.
        :return: A list of size 1 of a sane default RoleRequest.
        """
        pass

    def get_requests(
        self, world_state: rc.WorldState, props
    ) -> List[tactic.RoleRequests]:
        """
        :return: A list of role requests for move skills needed
        """

        if world_state and world_state.ball.visible:
            wall_pts = find_wall_pts(self.num_wallers, world_state)

            # assign move skill params and cost funcs to each waller
            for i in range(self.num_wallers):
                self.move_list[i].skill.target_point = wall_pts[i]
                self.move_list[i].skill.face_point = world_state.ball.pos
                self.cost_list[i].wall_pt = wall_pts[i]

        # create RoleRequest for each SkillEntry
        role_requests = {
            self.move_list[i]: [role.RoleRequest(role.Priority.HIGH, False, self.cost_list[i])]
            for i in range(self.num_wallers)
        }

        return role_requests

    def tick(self, role_results: tactic.RoleResults) -> List[tactic.SkillEntry]:
        """
        :return: A list of skills depending on which roles are filled
        """

        # create list of skills based on if RoleResult exists for SkillEntry 
        skills = [
            move_skill_entry
            for move_skill_entry in self.move_list
            if role_results[move_skill_entry][0]
        ]
        
        return skills

    def is_done(self, world_state):
        """
        :return boolean indicating if tactic is done
        """
        for move_skill in self.move_list:
            if not move_skill.skill.is_done(world_state):
                return False
        return True
