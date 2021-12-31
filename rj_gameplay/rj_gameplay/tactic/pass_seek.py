from dataclasses import dataclass
from typing import (
    List,
    Optional,
    Tuple,
    Optional,
    Callable,
)

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

import stp.global_parameters as global_parameters


def seek_heuristic(
    point: Tuple[float, float], world_state: Tuple[rc.WorldState]
) -> float:
    """
    func to find a point to move to
    """
    cost = 0
    for robot in world_state.their_robots:
        cost -= np.linalg.norm(np.array(point) - np.array(robot.pose[0:2]))
    cost -= 7 * point[1]
    return cost


def restart_seek(
    point: Tuple[float, float], world_state: Tuple[rc.WorldState]
) -> float:
    """
    func to find a point to move to
    robot "most open" @ point found by optimizer
    """
    cost = 0
    pt = np.array(point)
    ball_pos = world_state.ball.pos
    # hard constraint to be far from ball
    if np.linalg.norm(pt - ball_pos) < 1:
        return 99
    # keep dist from other robots (more dist = lower cost)
    for robot in world_state.their_robots:
        cost -= np.linalg.norm(pt - robot.pose[0:2])
    for robot in world_state.our_robots:
        cost -= np.linalg.norm(pt - robot.pose[0:2])

    # stay toward center
    cost += 10 * np.abs(point[0])

    # move upfield (more upfield = lower cost)
    cost -= 10 * point[1]
    return cost


def build_seek_function(target):
    def seek_heuristic(
        point: Tuple[float, float], world_state: Tuple[rc.WorldState]
    ) -> float:
        nonlocal target
        target = np.array(target)
        point = np.array(point)

        avoid_ball_cost = 0
        if world_state.ball.visible:
            goal_pos = np.array([0, world_state.field.length_m])
            ball_to_goal_vec = goal_pos - world_state.ball.pos
            ball_to_goal_dist = np.linalg.norm(ball_to_goal_vec)
            ball_to_goal_vec /= ball_to_goal_dist

            ball_to_goal_perp = np.array([-ball_to_goal_vec[1], ball_to_goal_vec[0]])
            perp_dist = np.dot(ball_to_goal_perp, point - world_state.ball.pos)

            range_diff = np.linalg.norm(goal_pos - point) - ball_to_goal_dist
            range_decay = np.exp(-range_diff) / (1.0 + np.exp(-range_diff))
            avoid_ball_cost = np.exp(-(perp_dist ** 2) * 30) * range_decay

        return np.linalg.norm(point - target) + avoid_ball_cost

    return seek_heuristic


class SeekCost(role.CostFn):
    """
    A cost function for how to choose a seeking robot
    TODO: Implement a better cost function
    """

    def __init__(self, target_point: np.ndarray):
        self.target_point = target_point

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

        return (
            np.linalg.norm(robot.pose[0:2] - self.target_point)
            / global_parameters.soccer.robot.max_speed
        )

    def unassigned_cost_fn(
        self,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        # TODO: Implement real unassigned cost function
        return role.BIG_STUPID_NUMBER_CONST_FOR_UNASSIGNED_COST_PLS_CHANGE


class Seek(tactic.ITactic):
    """
    A pass seeking tactic which tries to get open based on seek heuristic
    Role chosen by SeekCost
    # TODO: make naming less arbitrary
    """

    def __init__(
        self,
        target_point: np.ndarray,
        seek_heuristic: Callable[[Tuple[float, float]], float],
        seeker_cost: role.CostFn,
    ):
        goal_pos = np.array([0, 9])
        self.move = tactic.SkillEntry(
            move.Move(target_point=target_point, face_point=goal_pos)
        )
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
        """Checks if we have the ball and returns the proper request
        :return: A list of size 1 of role requests
        """

        role_requests: tactic.RoleRequests = {}

        move_request = role.RoleRequest(role.Priority.LOW, False, self.cost)
        role_requests[self.move] = [move_request]

        return role_requests

    def tick(
        self,
        world_state: rc.WorldState,
        role_results: tactic.RoleResults,
    ) -> List[tactic.SkillEntry]:
        """
        :return: A list of size 1 skill depending on which role is filled
        """
        goal_pos = np.array([0, world_state.field.length_m])
        self.move.skill.face_point = goal_pos
        self.move.skill.target_point = optimizer.find_seek_point(
            self.seek_heuristic, world_state
        )
        move_result = role_results[self.move]

        if move_result and move_result[0].is_filled():
            self.cost.locked_bot = move_result[0].role.robot
            return [self.move]
        return []

    def is_done(self, world_state):
        return self.move.skill.is_done(world_state)
