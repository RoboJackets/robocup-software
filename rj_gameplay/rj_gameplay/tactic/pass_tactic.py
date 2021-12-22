from dataclasses import dataclass
from typing import List, Optional
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar

import stp.rc as rc
import stp.tactic as tactic
import stp.role as role

import rj_gameplay.eval
import rj_gameplay.skill as skills
from rj_gameplay.skill import pivot_kick, receive, capture
import stp.skill as skill
import numpy as np
from math import atan2

import stp.global_parameters as global_parameters


class PassToClosestReceiver(role.CostFn):
    """
    A cost function for how to choose a robot to pass to
    """
    def __init__(self,
                 target_point: Optional[np.ndarray] = None,
                 passer_robot: rc.Robot = None):
        self.target_point = target_point
        self.passer_robot = passer_robot

    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        if robot is None or self.target_point is None:
            return 1e9
        # TODO (#1669)
        if not robot.visible:
            return 1e9
        if self.passer_robot is not None and robot.id == self.passer_robot.id:
            # can't pass to yourself
            return 1e9

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


class FindClosestPasser(role.CostFn):
    """
    A cost function for how to choose a robot that will pass
    TODO: Implement a better cost function
    """
    def __init__(self):
        pass

    def __call__(self,
                robot:rc.Robot,
                prev_result:Optional["RoleResult"],
                world_state:rc.WorldState) -> float:
        if robot.has_ball_sense:
            return 0

        # closest to ball is passer
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
    CURRENTLY NOT USED
    """
    def __init__(self,
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
            return 1e9
        # TODO (#1669)
        if not robot.visible:
            return 1e9
        if self.passer_robot is not None and robot.id == self.passer_robot.id:
            # can't pass to yourself
            return 1e9

        if self.passer_robot is not None and robot.id != self.passer_robot.id:
            pass_dist = np.linalg.norm(passer_robot.pose[0:2] -
                                       robot.pose[0:2])
            goal_to_receiver = np.linalg.norm(robot.pose[0:2] -
                                              rc.Field.their_goal_loc)
            cost = 0
            for enemy in world_state.their_robots:
                cost -= 10 * np.linalg.norm(enemy.pose[0:2] - robot.pose[0:2])
        else:
            return 1e9

    def unassigned_cost_fn(
        self,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        #TODO: Implement real unassigned cost function
        return role.BIG_STUPID_NUMBER_CONST_FOR_UNASSIGNED_COST_PLS_CHANGE


class PassToBestReceiver(role.CostFn):
    """
    A cost function for how to choose a robot to pass to
    TODO: improve this cost function

    """
    def __init__(self, passer_robot: rc.Robot = None):
        self.passer_robot = passer_robot
        self.chosen_receiver = None

    def __call__(self, robot: rc.Robot, prev_result: Optional["RoleResult"],
                 world_state: rc.WorldState) -> float:

        if robot is None:
            return 1e9

        if self.chosen_receiver is not None:
            if self.chosen_receiver.id == robot.id:
                return 0

        assert self.passer_robot is not None

        # can't pass to yourself
        if robot.id == self.passer_robot.id:
            return 1e9

        """
        if world_state is not None:
            min_dist = 999
            for our_robot in world_state.our_robots:
                dist = np.linalg.norm(world_state.ball.pos -
                                      our_robot.pose[0:2])
                if min_dist > dist:
                    min_dist = dist
        """

        angle_threshold = 5
        dist_threshold = 1.5
        backpass_punish_weight = 0.5

        pass_dist = np.linalg.norm(self.passer_robot.pose[0:2] -
                                   robot.pose[0:2])

        cost = 0
        #distance from their goal to receiver
        goal_to_receiver = np.linalg.norm(robot.pose[0:2] -
                                          world_state.field.their_goal_loc)
        #distance from their goal to passer
        goal_to_passer = np.linalg.norm(self.passer_robot.pose[0:2] -
                                        world_state.field.their_goal_loc)
        #Add cost for backwards passes scaled by backpass_punish_weight
        cost += (goal_to_receiver - goal_to_passer) * backpass_punish_weight
        for enemy in world_state.their_robots:
            passer_to_enemy = np.linalg.norm(enemy.pose[0:2] -
                                             self.passer_robot.pose[0:2])
            if passer_to_enemy <= pass_dist:
                vec_to_passer = robot.pose[0:2] - self.passer_robot.pose[0:2]
                vec_to_enemy = enemy.pose[0:2] - self.passer_robot.pose[0:2]
                angle = np.degrees(
                    abs(
                        atan2(np.linalg.det([vec_to_passer, vec_to_enemy]),
                              np.dot(vec_to_passer, vec_to_enemy))))
                if angle < angle_threshold:
                    return 1e9
            enemy_to_receiver = np.linalg.norm(robot.pose[0:2] -
                                               enemy.pose[0:2])
            if enemy_to_receiver < dist_threshold:
                cost += (dist_threshold - enemy_to_receiver)**2
        return cost


    def unassigned_cost_fn(
        self,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        #TODO: Implement real unassigned cost function
        return role.BIG_STUPID_NUMBER_CONST_FOR_UNASSIGNED_COST_PLS_CHANGE

class PickThisRobot(role.CostFn):
    """
    Cost fn that ensures one robot is always picked for some role, during the lifetime of this tactic.
    """
    def __init__(self, robot: rc.Robot):
        self.my_robot = robot

    def __call__(self, robot: rc.Robot, prev_result: Optional["RoleResult"],
                 world_state: rc.WorldState) -> float:

        if robot is None or self.my_robot is None:
            return 1e9
        if self.my_robot.id == robot.id:
            return 0
        return 1e9

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

    def __init__(self):
        # self.receiver_cost = PassToBestReceiver()
        # self.passer_cost = FindClosestPasser()

        # set after pass in motion
        self.receiver_robot = None
        self.passer_robot = None

        self.receive_se = None
        self.pivot_kick_se = None

        self.kick_done = False

    def compute_props(self):
        pass

    def create_request(self, **kwargs) -> role.RoleRequest:
        """Creates a sane default RoleRequest.
        :return: A list of size 1 of a sane default RoleRequest.
        """
        pass

    # to be called once on init
    # TODO: put in useful util file
    def find_min_cost_robot(self, world_state, cost_fn):
        min_cost = 1e9
        min_robot = None
        for robot in world_state.our_robots:
            curr_cost = cost_fn(robot, None, world_state)
            if curr_cost < min_cost:
                min_cost = curr_cost
                min_robot = robot
        return min_robot 

    def get_requests(
            self, world_state:rc.WorldState, props) -> List[tactic.RoleRequests]: # TODO: it doesn't return a List
        """ Checks if we have the ball and returns the proper request
        :return: A list of size 2 of role requests
        """

        role_requests: tactic.RoleRequests = {}

        if world_state is None:
            return []

        # pivot kick --> receive
        if self.passer_robot is None and self.receiver_robot is None:
            # on init, pick a passer and receiver
            passer_cost = FindClosestPasser()
            self.passer_robot = self.find_min_cost_robot(world_state, passer_cost)

            receiver_cost = PassToClosestReceiver(target_point=world_state.ball.pos, passer_robot=self.passer_robot)
            self.receiver_robot = self.find_min_cost_robot(world_state, receiver_cost)

        if self.passer_robot is not None and self.receiver_robot is not None:
            if not self.kick_done:
                # fill request for selected passer
                if self.pivot_kick_se is None:
                    self.pivot_kick_se = tactic.SkillEntry(pivot_kick.PivotKick(
                        robot=self.passer_robot,
                        target_point=self.receiver_robot.pose[:2], 
                        pivot_point=world_state.ball.pos))

                if self.pivot_kick_se.skill.is_done(world_state):
                    self.kick_done = True

                # have to update these every tick because receiver, world_state change positions
                # TODO: should pass these into the tick() method of every skill, change STP
                self.pivot_kick_se.skill.target_point=self.receiver_robot.pose[:2]
                self.pivot_kick_se.skill.pivot_point=world_state.ball.pos
                self.pivot_kick_se.skill.kick_speed=1.0

                passer_request = role.RoleRequest(role.Priority.HIGH, True, PickThisRobot(self.passer_robot))
                role_requests[self.pivot_kick_se] = [passer_request]
            else:
                # wait until kick happens, then request receiver
                if self.receive_se is None:
                    self.receive_se = tactic.SkillEntry(receive.Receive(
                        robot=self.receiver_robot
                    ))

                receive_request = role.RoleRequest(role.Priority.HIGH, True, PickThisRobot(self.receiver_robot))
                role_requests[self.receive_se] = [receive_request]

        return role_requests

    def tick(self, world_state: rc.WorldState,
             role_results: tactic.RoleResults) -> List[tactic.SkillEntry]:
        """
        :return: A list of size 1 or 2 skills depending on which roles are filled and state of aiming
        TODO: Come up with better timings for starting receive
        """
        skills = []
        
        pivot_result = role_results[self.pivot_kick_se]
        receive_result = role_results[self.receive_se]

        if pivot_result and pivot_result[0].is_filled():
            skills.append(self.pivot_kick_se)

        if receive_result and receive_result[0].is_filled():
            skills.append(self.receive_se)

        return skills

    def is_done(self, world_state:rc.WorldState):

        return self.kick_done

        if self.receive_se is not None:
            return self.receive_se.skill.is_done(world_state)
        return False
