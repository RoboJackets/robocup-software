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
from stp.utils.constants import RobotConstants

class wall_cost(role.CostFn):
    """
    Cost function for role request.
    """
    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState
    ) -> float:

        return 0.0

def my_robot_assigner(num_robots: int, mark_pt: np.ndarray, def_pt: np.ndarray, world_state: rc.WorldState):

    # [robot_id] = [rc.Robot, pt]
    assignments = {}

    for wall_pt in find_wall_pts(num_robots, mark_pt, def_pt):
        robot = robot_to_wall_pt(wall_pt, world_state, assignments)

        assignments[robot.id] = (robot, wall_pt)

    return assignments

def robot_to_wall_pt(wall_pt: np.ndarray, world_state: rc.WorldState, assignments):
    # TODO: dict type here?
    """
    A function that chooses which robot to move to a specific wall pt.
    """
    min_robot = None
    min_dist = float('inf')
    for robot in world_state.our_robots:
        # no duplicates
        if robot.id in assignments: 
            continue

        robot_pt = robot.pose[0:2]
        dist = np.linalg.norm(robot_pt - wall_pt)
        if dist < min_dist:
            min_dist = dist
            min_robot = robot

    return min_robot

def find_wall_pts(num_robots: int, mark_pt: np.ndarray, def_pt: np.ndarray):
    """
    # TODO: fill out multiple wall_pts
    mid_pt = (mark_pt + def_pt) / 2
    wall_pts = []
    for i in range(num_robots):
        wall_pts.append(i)
    return wall_pts
    """
    return [mark_pt, def_pt]

class WallTactic(tactic.ITactic):

    def __init__(self, 
            num_robots: int, 
            mark_pt: np.ndarray = None, 
            def_pt: np.ndarray = None,
        ):

        self.num_robots = num_robots
        # these will be defaulted to world_state on get_requests(), which is ticked by play
        self.mark_pt = mark_pt 
        self.def_pt = def_pt

        self.move_list = [
            tactic.SkillEntry(move.Move()) 
            for _ in range(num_robots)
        ]

        self.wall_pts = None
        self.cost = wall_cost()
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

        if world_state:
            self.world_state = world_state
            if self.mark_pt is None:
                self.mark_pt = world_state.ball.pos
            if self.def_pt is None:
                self.def_pt = world_state.field.our_goal_loc

        role_requests = {
            move_skill_entry: [role.RoleRequest(role.Priority.LOW, False, self.cost)] 
            for move_skill_entry in self.move_list
        }

        return role_requests

    def tick(self, role_results: tactic.RoleResults) -> List[tactic.SkillEntry]:
        """
        :return: A list of size 1 skill depending on which role is filled
        """

        # TODO: make this stanza less ugly
        # TODO: make assignment only happen ONCE
        # TODO: make assignment have no duplicates
        pt_to_robot = my_robot_assigner(self.num_robots, self.mark_pt, self.def_pt, self.world_state)
        i = 0
        for robot, pt in pt_to_robot.values():
            # assigns target point to each move skill
            self.move_list[i].skill.robot = robot
            self.move_list[i].skill.target_point = pt
            print("-"*80)
            print(robot.id, pt)
            i+=1

        skills = [
            move_skill_entry
            for move_skill_entry in self.move_list
            if role_results[move_skill_entry][0]
        ]
        
        return skills

    def is_done(self, world_state):
        for move_skill in self.move_list:
            if not move_skill.skill.is_done(world_state):
                return False
        return True

