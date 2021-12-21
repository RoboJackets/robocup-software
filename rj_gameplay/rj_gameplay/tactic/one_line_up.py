"""Contains the stub for the move tactic. """

from dataclasses import dataclass
from typing import List, Optional, Any
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar, Any

import stp.action as action
import stp.rc as rc
import stp.tactic as tactic
import stp.role as role

import rj_gameplay.eval
import rj_gameplay.skill as skills
from rj_gameplay.skill import shoot, capture, move
import stp.skill as skill
import numpy as np


class one_lineup_cost(role.CostFn):
    """
    A cost function for how to choose a striker
    TODO: Implement a better cost function
    """
    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        # if prev_result is not None and prev_result.role.robot == robot:
        #     return prev_result.role.robot
        # return robot.id
        return 0

    def unassigned_cost_fn(
        self,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        #TODO: Implement real unassigned cost function
        return role.BIG_STUPID_NUMBER_CONST_FOR_UNASSIGNED_COST_PLS_CHANGE

class one_lineup_constraint(role.ConstraintFn):
    """Protocol for ConstraintFn. """

    def __init__(self, action_client_dict: Dict[Type[Any], List[Any]],
                 robot_id):
        self.robot_id = robot_id

    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> bool:
        return robot.id == self.robot_id

class OneLineUp(tactic.ITactic):
    """
    A striker tactic which captures then shoots the ball
    """


    def __init__(self, action_client_dict: Dict[Type[Any], List[Any]],
                 robot_id):
        self.left_x = 1.0
        self.right_x = -1.5
        self.start_y = 2.0
        self.move_right = tactic.SkillEntry(
            move.Move(action_client_dict,
                      target_point=np.array([self.right_x, self.start_y])))
        self.move_left = tactic.SkillEntry(
            move.Move(action_client_dict,
                      target_point=np.array([self.left_x, self.start_y])))
        self.cost = one_lineup_cost()
        self.robot_id = robot_id
        self.constraint = one_lineup_constraint(self.robot_id)
        self.world_state = None

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
        """ Checks if we have the ball and returns the proper request
        :return: A list of size 1 of role requests
        """
        self.world_state = world_state
        role_requests: tactic.RoleRequests = {}

        move_request = role.RoleRequest(role.Priority.HIGH, True, self.cost, self.constraint)
        # has_ball = True
        # for robot in world_state.our_robots:
        #     if robot.has_ball_sense:
        #         has_ball = True
        # if has_ball:
        #     role_requests[self.shoot] = [striker_request]
        #     role_requests[self.capture] = []
        # else:
        #     role_requests[self.capture] = [striker_request]
        #     role_requests[self.shoot] = []
        # role_requests
        if world_state.our_robots[self.robot_id].pose[0] <= -1.49:
            role_requests = {self.move_left :[move_request]}
        else:
            role_requests = {self.move_right :[move_request]}

        return role_requests

    def tick(self, world_state: rc.WorldState,
             role_results: tactic.RoleResults) -> List[tactic.SkillEntry]:
        """
        :return: A list of size 1 skill depending on which role is filled
        """
        # capture_result: tactic.RoleResults
        # capture_result = role_results[self.capture]
        # shoot_result = role_results[self.shoot]
        if self.world_state.our_robots[self.robot_id].pose[0] <= -1.49:
            move_result = role_results[self.move_left]
            return[self.move_left]
        else:
            move_result = role_results[self.move_right]
            print(role_results)
            return[self.move_right]
        return []

    def is_done(self, world_state):
        return self.move_right.skill.is_done(world_state) and self.move_left.skill.is_done(world_state)
