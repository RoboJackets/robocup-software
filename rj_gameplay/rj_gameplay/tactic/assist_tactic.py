from dataclasses import dataclass
from typing import List, Optional
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar

import stp.action as action
import stp.rc as rc
import stp.tactic as tactic
import stp.role as role

import rj_gameplay.eval
import rj_gameplay.skill as skills
from rj_gameplay.skill import pivot_kick, receive
import stp.skill as skill
from math import atan2
import numpy as np

def find_striker_cost(robot:rc.Robot, world_state: rc.WorldState):
    cost = 0
    goal_loc = world_state.field.their_goal_loc
    kicker = world_state.our_robots[robot.id]
    left_end = np.array([-0.5, world_state.field.length_m])
    right_end = np.array([0.5, world_state.field.length_m])
    dist_to_goal = np.linalg.norm(goal_loc - kicker.pose[0:2])

    if dist_to_goal > 5:
        return 9999
    else:
        u_vec_kicker_left = (left_end - kicker.pose[0:2]) / np.linalg.norm(left_end - kicker.pose[0:2])
        u_vec_kicker_right = (right_end - kicker.pose[0:2]) / np.linalg.norm(right_end - kicker.pose[0:2])
        shoot_range = atan2(np.linalg.det([u_vec_kicker_left, u_vec_kicker_right]), np.dot(u_vec_kicker_left, u_vec_kicker_right))
        #TODO find weight
        cost -= shoot_range * 10
        for opp_robot in world_state.their_robots:
            u_vec_kicker_opp = (opp_robot.pose[0:2] - kicker.pose[0:2]) / np.linalg.norm(opp_robot.pose[0:2] - kicker.pose[0:2])

            if np.dot(u_vec_kicker_left, u_vec_kicker_right) < np.dot(u_vec_kicker_left, u_vec_kicker_opp) and \
                np.dot(u_vec_kicker_left, u_vec_kicker_right) < np.dot(u_vec_kicker_right, u_vec_kicker_opp):
                cost += (world_state.field.length_m - opp_robot.pose[1]) / (world_state.field.length_m - kicker.pose[1]) *2

        
        return cost


class StrikerCost(role.CostFn):
    """
    A cost function for how to choose a robot to pass to
    TODO: Implement a better cost function
    """

    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:
    
        return find_striker_cost(robot, world_state)

class CaptureCost(role.CostFn):
    """
    A cost function for capturing ball
    """
    def __call__(self,
                robot:rc.Robot,
                prev_result:Optional["RoleResult"],
                world_state:rc.WorldState) -> float:
        if robot.has_ball_sense:
            return 0
        else:
            robot_pos = robot.pose[0:2]
            ball_pos = world_state.ball.pos[0:2]
            dist_to_ball = np.linalg.norm(ball_pos - robot_pos)
            return dist_to_ball



class AssistTactic(tactic.ITactic):
    """
    A passing tactic which captures then passes the ball
    """

    def __init__(self, striker_loc:np.ndarray):
        self.striker_loc = striker_loc
        self.pivot_kick = tactic.SkillEntry(pivot_kick.PivotKick(target_point = striker_loc, chip=False, kick_speed=4.0))
        self.receive = tactic.SkillEntry(receive.Receive())
        self.striker_cost = StrikerCost()
        self.capture_cost = CaptureCost()
        
    def compute_props(self):
        pass

    def create_request(self, **kwargs) -> role.RoleRequest:
        """Creates a sane default RoleRequest.
        :return: A list of size 1 of a sane default RoleRequest.
        """
        pass

    def get_requests(
        self, world_state:rc.WorldState, props) -> List[tactic.RoleRequests]:
        """ Checks if we have the ball and returns the proper request
        :return: A list of size 2 of role requests
        """

        role_requests: tactic.RoleRequests = {}

        pass_request = role.RoleRequest(role.Priority.HIGH, True, self.capture_cost)
        role_requests[self.pivot_kick] = [pass_request]
        receive_request = role.RoleRequest(role.Priority.HIGH, True, self.striker_cost)
        role_requests[self.receive] = [receive_request]

        return role_requests

    def tick(self, role_results:tactic.RoleResults, world_state:rc.WorldState) -> List[tactic.SkillEntry]:
        """
        :return: A list of size 1 or 2 skills depending on which roles are filled and state of aiming
        TODO: Come up with better timings for starting receive
        """
        pivot_result = role_results[self.pivot_kick]
        receive_result = role_results[self.receive]

        if pivot_result and receive_result and pivot_result[0].is_filled() and receive_result[0].is_filled():
            self.pivot_kick.skill.target_point = np.array(receive_result[0].role.robot.pose[0:2])
            if self.pivot_kick.skill.kick.is_done(world_state):
                return [self.pivot_kick, self.receive]
            else:
                return [self.pivot_kick]
        elif pivot_result and pivot_result[0].is_filled():
            return [self.pivot_kick]
        return []

    def is_done(self, world_state:rc.WorldState):
        return self.receive.skill.is_done(world_state)
