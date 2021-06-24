from dataclasses import dataclass
from typing import List, Optional
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar

import stp.action as action
import stp.rc as rc
import stp.tactic as tactic
import stp.role as role

import rj_gameplay.eval
import rj_gameplay.skill as skills
from rj_gameplay.skill import pivot_kick, receive, line_kick
import stp.skill as skill
import numpy as np

class ClearerCost(role.CostFn):
    """
    A cost function for how to choose a robot that will clears
    """
    def __call__(self,
                robot:rc.Robot,
                prev_result:Optional["RoleResult"],
                world_state:rc.WorldState) -> float:
        if not robot.visible:
            return 99
        else:
            return np.linalg.norm(world_state.ball.pos - np.array(robot.pose[0:2]))

class Clear(tactic.ITactic):
    """
    A passing tactic which captures then passes the ball
    """

    def __init__(self, target_point:np.ndarray, kick_speed=3.0, chip=False):
        self.target_point = target_point
        self.kick = tactic.SkillEntry(line_kick.LineKickSkill(None, target_point=target_point, chip=chip, kick_speed=kick_speed))
        self.clearer_cost = ClearerCost()

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

        clearer_request = role.RoleRequest(role.Priority.MEDIUM, True, self.clearer_cost)
        role_requests[self.kick] = [clearer_request]

        return role_requests

    def tick(self, role_results:tactic.RoleResults, world_state:rc.WorldState) -> List[tactic.SkillEntry]:
        """
        :return: A list of size 1 or 2 skills depending on which roles are filled and state of aiming
        TODO: Come up with better timings for starting receive
        """
        clearer_result = role_results[self.kick]
        if clearer_result and clearer_result[0].is_filled():
            return [self.kick]
        return []

    def is_done(self, world_state:rc.WorldState):
        # if self.kick.skill.is_done(world_state):
        #     print("WOWOWWWOWOWO")
        return self.kick.skill.is_done(world_state)
