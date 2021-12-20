from dataclasses import dataclass
from typing import List, Optional, Any
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar, Any

import stp.action as action
import stp.rc as rc
import stp.tactic as tactic
import stp.role as role

import rj_gameplay.eval
import rj_gameplay.skill as skills
from rj_gameplay.skill import pivot_kick, receive
import stp.skill as skill
import numpy as np

import stp.global_parameters as global_parameters


class PassToClosestReceiver(role.CostFn):
    """
    A cost function for how to choose a robot to pass to
    """
    def __init__(self, action_client_dict: Dict[Type[Any], List[Any]],
                 target_point: Optional[np.ndarray] = None,
                 passer_robot: rc.Robot = None):
        self.target_point = target_point
        self.passer_robot = passer_robot
        self.chosen_receiver = None

    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        if robot is None or self.target_point is None:
            return 99
        # TODO (#1669)
        if not robot.visible:
            return 99
        if self.passer_robot is not None and robot.id == self.passer_robot.id:
            # can't pass to yourself
            return 99
        if self.chosen_receiver is not None and self.chosen_receiver.id == robot.id:
            return -99

        # always pick closest receiver
        raw_dist = np.linalg.norm(robot.pose[0:2] - self.target_point)
        return raw_dist / global_parameters.soccer.robot.max_speed

    def unassigned_cost_fn(
        self,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        #TODO: Implement real unassigned cost function
        return role.BIG_STUPID_NUMBER_CONST_FOR_UNASSIGNED_COST_PLS_CHANGE


class PasserCost(role.CostFn):
    """
    A cost function for how to choose a robot that will pass
    TODO: Implement a better cost function
    """

    def __call__(self,
                robot:rc.Robot,
                prev_result:Optional["RoleResult"],
                world_state:rc.WorldState) -> float:
        if robot.has_ball_sense:
            return 0
        else:
            # closest to ball
            return np.linalg.norm(world_state.ball.pos - robot.pose[0:2])

    def unassigned_cost_fn(
        self,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        #TODO: Implement real unassigned cost function
        return role.BIG_STUPID_NUMBER_CONST_FOR_UNASSIGNED_COST_PLS_CHANGE


class PassToOpenReceiver(role.CostFn):
    """
    A cost function for how to choose a robot to pass to
    TODO: Implement a better cost function
    CURRENTLY NOT READY FOR USE
    """
    def __init__(self, action_client_dict: Dict[Type[Any], List[Any]],
                 target_point: Optional[np.ndarray] = None,
                 passer_robot: rc.Robot = None):
        self.target_point = target_point
        self.passer_robot = passer_robot
        self.chosen_receiver = None

    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        if robot is None or self.target_point is None:
            return 99
        # TODO (#1669)
        if not robot.visible:
            return 99
        if self.passer_robot is not None and robot.id == self.passer_robot.id:
            # can't pass to yourself
            return 99
        if self.chosen_receiver is not None and self.chosen_receiver.id == robot.id:
            return -99

        # TODO: pick "most open" pass
        cost = 0
        for enemy in world_state.their_robots:
            cost -= 10 * np.linalg.norm(enemy.pose[0:2] - robot.pose[0:2])

        # TODO: should be dist in sec
        # raw_dist = np.linalg.norm(robot.pose[0:2] - self.target_point)
        # cost = cost + raw_dist
        return cost

        if robot is None or self.target_point is None:
            return 99
        # TODO (#1669)
        if not robot.visible:
            return 99
        if self.passer_robot is not None and robot.id == self.passer_robot.id:
            # can't pass to yourself
            return 99

        # TODO: pick "most open" pass
        cost = 0 
        for enemy in world_state.their_robots:
            cost -= 10*np.linalg.norm(enemy.pose[0:2] - robot.pose[0:2])

        # TODO: should be dist in sec
        # raw_dist = np.linalg.norm(robot.pose[0:2] - self.target_point) 
        # cost = cost + raw_dist
        return cost

    def unassigned_cost_fn(
        self,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        #TODO: Implement real unassigned cost function
        return role.BIG_STUPID_NUMBER_CONST_FOR_UNASSIGNED_COST_PLS_CHANGE

class Pass(tactic.ITactic):
    """
    A passing tactic which captures then passes the ball
    """

    def __init__(self, action_client_dict: Dict[Type[Any], List[Any]], target_point: np.ndarray, passer_cost: role.CostFn,
                 receiver_cost: role.CostFn):
        self.target_point = target_point
        self.pivot_kick = tactic.SkillEntry(
            pivot_kick.PivotKick(robot=None,
                                 target_point=target_point,
                                 chip=False,
                                 kick_speed=4.0))
        self.receive = tactic.SkillEntry(receive.Receive())
        self.receiver_cost = receiver_cost
        self.passer_cost = passer_cost

    def compute_props(self):
        pass

    def create_request(self, **kwargs) -> role.RoleRequest:
        """Creates a sane default RoleRequest.
        :return: A list of size 1 of a sane default RoleRequest.
        """
        pass

    def find_potential_receiver(self, world_state: rc.WorldState) -> rc.Robot:
        cost = float('inf')
        receive_robot = None
        for robot in world_state.our_robots:
            curr_cost = self.receiver_cost(robot, None, world_state)
            if curr_cost < cost:
                cost = curr_cost
                receive_robot = robot
        return receive_robot

    def get_requests(
        self, world_state:rc.WorldState, props) -> List[tactic.RoleRequests]:
        """ Checks if we have the ball and returns the proper request
        :return: A list of size 2 of role requests
        """

        role_requests: tactic.RoleRequests = {}

        passer_request = role.RoleRequest(role.Priority.MEDIUM, True,
                                          self.passer_cost)
        role_requests[self.pivot_kick] = [passer_request]
        if self.pivot_kick.skill.is_done(world_state):
            receive_request = role.RoleRequest(role.Priority.MEDIUM, True,
                                               self.receiver_cost)
            role_requests[self.receive] = [receive_request]
        else:
            passer_request = role.RoleRequest(role.Priority.HIGH, True,
                                              self.passer_cost)
            role_requests[self.pivot_kick] = [passer_request]

        self.receiver_cost.passer_robot = self.pivot_kick.skill.robot

        return role_requests

    def tick(self, world_state: rc.WorldState,
             role_results: tactic.RoleResults) -> List[tactic.SkillEntry]:
        """
        :return: A list of size 1 or 2 skills depending on which roles are filled and state of aiming
        TODO: Come up with better timings for starting receive
        """
        pivot_result = role_results[self.pivot_kick]
        receive_result = role_results[self.receive]
        if receive_result and receive_result[0].is_filled():
            self.passer_cost.chosen_receiver = receive_result[0].role.robot
            self.pivot_kick.skill.target_point = np.array(receive_result[0].role.robot.pose[0:2])
            self.passer_cost.potential_receiver = receive_result[0].role.robot
            return [self.receive]
        elif pivot_result and pivot_result[0].is_filled():
            potential_receiver = self.find_potential_receiver(world_state)
            self.pivot_kick.skill.target_point = np.array(
                [potential_receiver.pose[0], potential_receiver.pose[1]])
            return [self.pivot_kick]
        return []

    def is_done(self, world_state:rc.WorldState):
        print(self.receive.skill.is_done(world_state))
        return self.receive.skill.is_done(world_state)
