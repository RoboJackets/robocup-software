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
from rj_gameplay.skill import move, receive  #, intercept
import stp.skill as skill
import numpy as np
# TODO: replace w/ global param server
from stp.utils.constants import RobotConstants, BallConstants
import stp.global_parameters as global_parameters
from stp.local_parameters import Param

# TODO: param server this const
MIN_WALL_RAD = 0
GOALIE_PCT_TO_BALL = 0.15


class goalie_cost(role.CostFn):
    """Cost function for role request. Want only the designated goalie to be selected.
    """
    def __call__(self, robot: rc.Robot, prev_result: Optional["RoleResult"],
                 world_state: rc.WorldState) -> float:

        if world_state.game_info is not None:
            # TODO: clear debug prints
            # print(world_state.game_info.our_restart)
            # print(world_state.game_info.goalie_id)

            # this is a hacky way of doing goalie, until sub is fixed
            # TODO: fix goalie_id sub in gameplay node
            if robot.id == 0:
                # if robot.id == world_state.game_info.goalie_id:
                return -1.0
        return 999.0


def get_goalie_pt(world_state: rc.WorldState) -> np.ndarray:
    """Finds point for goalie to best be in to block a shot.
    :return numpy point
    """
    # TODO: param server any constant from stp/utils/constants.py (this includes BallConstants)
    ball_pt = world_state.ball.pos
    goal_pt = world_state.field.our_goal_loc

    dir_vec = (ball_pt - goal_pt) / np.linalg.norm(ball_pt - goal_pt)
    # get in-between ball and goal, staying behind wall
    dist_from_goal = min(
        GOALIE_PCT_TO_BALL * np.linalg.norm(ball_pt - goal_pt),
        MIN_WALL_RAD - RobotConstants.RADIUS * 2.1)
    mid_pt = goal_pt + (dir_vec * dist_from_goal)
    return mid_pt


# TODO: replace with intercept
def get_block_pt(world_state: rc.WorldState, my_pos: np.ndarray) -> np.ndarray:
    pos = world_state.ball.pos
    vel = world_state.ball.vel

    block_pt = np.array([(my_pos[1] - pos[1]) / vel[1] * vel[0] + pos[0],
                         my_pos[1]])

    return block_pt


class GoalieTactic(tactic.ITactic):
    def __init__(self):

        # create move SkillEntry
        self.move_se = tactic.SkillEntry(move.Move())

        # TODO: replace w/ intercept
        self.receive_se = tactic.SkillEntry(receive.Receive())

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
        global MIN_WALL_RAD
        """
        :return: A list of role requests for move skills needed
        """

        # TODO: this calculation is copy-pasted from wall_tactic
        # put into common param file: https://www.geeksforgeeks.org/global-keyword-in-python/

        # dist is slightly greater than penalty box bounds
        box_w = world_state.field.penalty_long_dist_m
        box_h = world_state.field.penalty_short_dist_m
        line_w = world_state.field.line_width_m
        MIN_WALL_RAD = RobotConstants.RADIUS + line_w + np.hypot(
            box_w / 2, box_h)

        role_requests = {}
        if world_state and world_state.ball.visible:
            ball_speed = np.linalg.norm(world_state.ball.vel)
            ball_dist = np.linalg.norm(world_state.field.our_goal_loc -
                                       world_state.ball.pos)

            if ball_speed < 1.0 and ball_dist < MIN_WALL_RAD - RobotConstants.RADIUS * 2.1:
                # if ball is slow and inside goalie box, collect it
                role_requests[self.receive_se] = [
                    role.RoleRequest(role.Priority.HIGH, True, self.role_cost)
                ]
            else:
                ball_to_goal_time = ball_dist / ball_speed
                if ball_speed > 0 and ball_to_goal_time < 2:
                    # if ball is moving and coming at goal, move laterally to block ball
                    self.move_se.skill.target_point = get_block_pt(
                        world_state, get_goalie_pt(world_state))
                    self.move_se.skill.face_point = world_state.ball.pos
                    role_requests[self.move_se] = [
                        role.RoleRequest(role.Priority.HIGH, True,
                                         self.role_cost)
                    ]
                else:
                    # else, track ball normally
                    self.move_se.skill.target_point = get_goalie_pt(
                        world_state)
                    self.move_se.skill.face_point = world_state.ball.pos
                    role_requests[self.move_se] = [
                        role.RoleRequest(role.Priority.HIGH, True,
                                         self.role_cost)
                    ]

        return role_requests

    def tick(self,
             role_results: tactic.RoleResults) -> List[tactic.SkillEntry]:
        """
        :return: A list of skills depending on which roles are filled
        """

        # create list of skills based on if RoleResult exists for SkillEntry
        skills = []

        move_result = role_results[self.move_se]
        if move_result and move_result[0].is_filled():
            skills.append(self.move_se)
        else:
            # move first, then receive
            receive_result = role_results[self.receive_se]
            if receive_result and receive_result[0].is_filled():
                skills.append(self.receive_se)

        return skills

    def is_done(self, world_state):
        """
        :return boolean indicating if tactic is done
        """
        # goalie tactic always active
        return False
