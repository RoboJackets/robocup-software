"""Tactic to produce goalie behavior, which tracks the ball, moves to block if a shot on goal is taken, and stays within the goalie box (generally)."""

from dataclasses import dataclass
from typing import List, Optional
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar

import stp.action as action
import stp.rc as rc
import stp.tactic as tactic
import stp.role as role

import rj_gameplay.eval
import rj_gameplay.skill as skills
from rj_gameplay.skill import move, receive, line_kick  # , intercept
import stp.skill as skill
import numpy as np
# TODO: replace w/ global param server
from stp.utils.constants import RobotConstants, BallConstants
import stp.global_parameters as global_parameters
from stp.local_parameters import Param

# TODO: param server this const
MIN_WALL_RAD = 0
GOALIE_PCT_TO_BALL = 0.15
DIST_TO_FAST_KICK = 7


class GoalieCost(role.CostFn):
    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:
        if world_state.game_info is not None:
            if robot.id == world_state.goalie_id:
                return 0.0

        return 10000000


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
        GOALIE_PCT_TO_BALL * np.linalg.norm(ball_pt - goal_pt), 1.0)
    mid_pt = goal_pt + (dir_vec * dist_from_goal)
    return mid_pt


def get_block_pt(world_state: rc.WorldState, my_pos: np.ndarray) -> np.ndarray:
    pos = world_state.ball.pos
    vel = world_state.ball.vel

    tangent = vel / (np.linalg.norm(vel) + 1e-6)

    # Find out where it would cross
    time_to_cross = np.abs(pos[1] / vel[1]) if np.abs(vel[1]) > 1e-6 else 0
    cross_x = pos[0] + vel[0] * time_to_cross
    cross_position = np.array([np.clip(cross_x, a_min=-0.6, a_max=0.6), 0.0])

    tangent = cross_position - pos
    tangent /= np.linalg.norm(tangent)
    block_pt = np.dot(tangent, my_pos - pos) * tangent + pos

    return block_pt


class GoalieTactic(tactic.ITactic):
    def __init__(self, brick=False):
        self.brick = brick

        # init skills
        self.move_se = tactic.SkillEntry(move.Move(ignore_ball=True))
        self.receive_se = tactic.SkillEntry(receive.Receive())
        self.pivot_kick_se = tactic.SkillEntry(
            line_kick.LineKickSkill(None,
                                    target_point=np.array([0.0, 6.0]),
                                    chip=True,
                                    kick_speed=5.5))

        # TODO: rename cost_list to role_cost in other gameplay files
        self.role_cost = GoalieCost()

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
        def track_utility() -> float:
            """
            :return: the utility of track action
            """
            if ball_speed == 0:
                ball_to_goal_time = 100
            else:
                ball_to_goal_time = ball_dist / ball_speed
            return ball_to_goal_time / 100

        def receive_utility() -> float:
            """
            :return: the utility of receive action
            """
            # this probably makes sense to stay hard coded like this
            # but maybe there is a better option?
            if ball_speed < .5 and (
                    abs(ball_pos[0]) < box_w / 2 + line_w + MAX_OOB
                    and ball_pos[1] < box_h + line_w + MAX_OOB
            ) and not world_state.game_info.is_stopped():
                return 1
            else:
                return 0

        def block_utility() -> float:
            """Increases as ball to goal time approaches 0.
            :return: the utility of block action
            """
            if ball_speed > 0 and np.dot(towards_goal,
                                         world_state.ball.vel) > 0.3:
                # if ball is moving and coming at goal, move laterally to block ball
                ball_to_goal_time = ball_dist / ball_speed
            else:
                ball_to_goal_time = 100
            return 1 - (ball_to_goal_time / 100)

        def clear_utility() -> float:
            """
            :return: the utility of clear action
            """
            # should clear if no open pass available
            # how do I check that?
            return 1

        def pass_utility() -> float:
            """
            :return: the utility of clear action
            """
            # should pass if open pass available
            # same problem as clear
            return 0

        def get_best_action():
            """Compares utility functions of all actions.
            :return: The best action based on the utility functions.
            """
            if world_state and world_state.ball.visible:
                # if tied earlier index will be selected
                utility_values = np.array([
                    pass_utility(),
                    clear_utility(),
                    track_utility(),
                    receive_utility(),
                    block_utility()
                ])

                utilities = np.array([
                    pass_utility, clear_utility, track_utility,
                    receive_utility, block_utility
                ])
                return utilities[np.argmax(utility_values)]
            else:
                return None

        # TODO: this calculation is copy-pasted from wall_tactic
        # variables
        # put into common param file: https://www.geeksforgeeks.org/global-keyword-in-python/

        # dist is slightly greater than penalty box bounds
        box_w = world_state.field.penalty_long_dist_m
        box_h = world_state.field.penalty_short_dist_m
        line_w = world_state.field.line_width_m
        # max out of box to cap for goalie
        MAX_OOB = RobotConstants.RADIUS

        ball_speed = np.linalg.norm(world_state.ball.vel)
        ball_pos = world_state.ball.pos
        ball_dist = np.linalg.norm(world_state.field.our_goal_loc - ball_pos)
        goal_pos = world_state.field.our_goal_loc
        towards_goal = goal_pos - ball_pos

        role_requests = {}
        best_action = get_best_action()

        if best_action is not None:

            # TODO (1724) remove this from the code
            if self.brick:
                self.move_se.skill.target_point = world_state.field.our_goal_loc
                self.move_se.skill.face_point = world_state.ball.pos
                role_requests[self.move_se] = [
                    role.RoleRequest(role.Priority.HIGH, True, self.role_cost)
                ]
                return role_requests

            if best_action is receive_utility:
                # if ball is stopped and inside goalie box, collect it
                self.move_se = tactic.SkillEntry(move.Move(ignore_ball=True))
                role_requests[self.receive_se] = [
                    role.RoleRequest(role.Priority.HIGH, True, self.role_cost)
                ]
            elif best_action is clear_utility:
                # clear
                # if ball has been stopped already, chip toward center field
                self.move_se = tactic.SkillEntry(move.Move(ignore_ball=True))
                self.pivot_kick_se.skill.target_point = np.array([0.0, 6.0])
                role_requests[self.pivot_kick_se] = [
                    role.RoleRequest(role.Priority.HIGH, True, self.role_cost)
                ]

            elif best_action is block_utility:
                # if ball is moving and coming at goal, move laterally to block ball
                # TODO (#1676): replace this logic with a real intercept planner
                goalie_pos = world_state.our_robots[
                    world_state.
                    goalie_id].pose[:
                                    2] if world_state.goalie_id is not None else np.array(
                                        [0., 0.])
                self.move_se.skill.target_point = get_block_pt(
                    world_state, goalie_pos)
                self.move_se.skill.face_point = world_state.ball.pos
                role_requests[self.move_se] = [
                    role.RoleRequest(role.Priority.HIGH, True, self.role_cost)
                ]
            elif best_action is track_utility:
                # track ball normally
                self.move_se.skill.target_point = get_goalie_pt(world_state)
                self.move_se.skill.face_point = world_state.ball.pos
                role_requests[self.move_se] = [
                    role.RoleRequest(role.Priority.HIGH, True, self.role_cost)
                ]
            elif best_action is pass_utility:
                # TODO : under current system, should assign pass tactic here
                pass

        # reminder : remove during testing
        if self.pivot_kick_se.skill.is_done(world_state):
            self.pivot_kick_se = tactic.SkillEntry(
                line_kick.LineKickSkill(None,
                                        target_point=np.array([0.0, 6.0]),
                                        chip=True,
                                        kick_speed=5.5))

        return role_requests

    def tick(self,
             role_results: tactic.RoleResults) -> List[tactic.SkillEntry]:
        """
        :return: A list of skills depending on which roles are filled
        """

        # create list of skills based on if RoleResult exists for SkillEntry
        skills = []

        move_result = role_results[self.move_se]
        receive_result = role_results[self.receive_se]
        pivot_kick_result = role_results[self.pivot_kick_se]

        # move skill takes priority
        if move_result and move_result[0].is_filled():
            skills.append(self.move_se)
        elif receive_result and receive_result[0].is_filled():
            skills.append(self.receive_se)
        elif pivot_kick_result and pivot_kick_result[0].is_filled():
            skills.append(self.pivot_kick_se)

        return skills

    def is_done(self, world_state):
        """
        :return boolean indicating if tactic is done
        """
        # goalie tactic always active
        return False
