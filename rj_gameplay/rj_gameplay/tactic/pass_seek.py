from dataclasses import dataclass
from typing import List, Optional
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar, Callable

import stp.action as action
import stp.rc as rc
import stp.tactic as tactic
import stp.role as role
import stp.utils.pass_seeker_optimizer as optimizer

import rj_gameplay.eval
import rj_gameplay.skill as skills
from rj_gameplay.skill import shoot, capture, move
import stp.skill as skill
import numpy as np

def seek_heuristic(point: Tuple[float, float], world_state: Tuple[rc.WorldState]) -> float:
    cost = 0
    for robot in world_state.their_robots:
        cost-= np.linalg.norm(np.array(point)-np.array(robot.pose[0:2]))
    cost -= 7 * point[1]
    return cost

class seek_cost(role.CostFn):
    """
    A cost function for how to choose a seeking robot
    TODO: Implement a better cost function
    """
    def __init__(self, target_point : np.ndarray):
        self.target_point = target_point

    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:
    
        if robot.id == 7:
            return 0.0
        else:
            return 1.0

class Seek(tactic.ITactic):
    """
    A pass seeking tactic which tries to get open
    """


    def __init__(self, target_point:np.ndarray, seek_heuristic:Callable[[Tuple[float, float]], float], seeker_cost:role.CostFn):
        self.move = tactic.SkillEntry(move.Move(target_point = target_point))
        self.cost = seeker_cost
        self.seek_heuristic = seek_heuristic
        
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

        role_requests: tactic.RoleRequests = {}

        move_request = role.RoleRequest(role.Priority.LOW, False, self.cost)
        role_requests[self.move] = [move_request]

        return role_requests

    def tick(self, role_results: tactic.RoleResults, world_state: rc.WorldState) -> List[tactic.SkillEntry]:
        """
        :return: A list of size 1 skill depending on which role is filled
        """
        # capture_result: tactic.RoleResults
        # capture_result = role_results[self.capture]
        # shoot_result = role_results[self.shoot]
        self.move.skill.target_point = optimizer.find_seek_point(self.seek_heuristic, world_state)
        move_result = role_results[self.move]

        if move_result and move_result[0].is_filled():
            return [self.move]
        return []

    def is_done(self, world_state):
        return self.move.skill.is_done(world_state)
