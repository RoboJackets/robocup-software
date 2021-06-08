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
    print("-"*80)
    print("called")

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
    """
    for robot in world_state.our_robots:
        if robot.id is 2: 
            if 2 not in assignments:
                return robot
        if robot.id is 1: 
            if 1 not in assignments:
                return robot
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

    print("wall_pt:", wall_pt)
    print("closest bot:", min_robot.id)
    return min_robot

def find_wall_pts(num_robots: int, mark_pt: np.ndarray, def_pt: np.ndarray):
    # TODO: introduce curvature (currently flat wall)

    # TODO: param server this const
    WALL_SPACING = RobotConstants.RADIUS / 2 # 1/4th robot diameter

    # set midpoint first
    mid_pt = (mark_pt + def_pt) / 2
    wall_pts = [mid_pt]

    # get dir, find perp vec to that
    dir_vec = (mark_pt - def_pt) / np.linalg.norm(mark_pt - def_pt)
    perp = np.array([dir_vec[1], -dir_vec[0]])

    print("find_wall_pts")
    print("mark_pt:", mark_pt)
    print("def_pt:", def_pt)

    # set wall points in middle out pattern, given perp vector and WALL_SPACING constant
    for i in range(num_robots-1):
        mult = i//2 + 1
        delta = (mult * (2 * RobotConstants.RADIUS + WALL_SPACING)) * perp
        if i % 2: delta = -delta
        wall_pts.append(mid_pt + delta)

    return wall_pts

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
        self.my_assignments = None
        
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
            self.mark_pt = world_state.ball.pos
            self.def_pt = world_state.field.our_goal_loc

            # TODO: make closest robots form wall, rather than setting on init
            """
            if self.my_assignments is not None:
                # clear old assignments
                for move_skill_entry in self.move_list:
                    robot = move_skill_entry.robot
                    move_skill_entry.skill.target_point = 
            """

            # TODO: make this stanza less ugly
            self.my_assignments = my_robot_assigner(self.num_robots, self.mark_pt, self.def_pt, self.world_state)
            i = 0
            for robot, pt in self.my_assignments.values():
                # assigns target point to each move skill
                self.move_list[i].skill.robot = robot
                self.move_list[i].skill.target_point = pt
                i+=1

            for se in self.move_list:
                print(se.skill.robot.id)
                print(se.skill.target_point)

            for move_skill_entry in self.move_list:
                move_skill_entry.skill.face_point = self.mark_pt

        role_requests = {
            move_skill_entry: [role.RoleRequest(role.Priority.LOW, False, self.cost)] 
            for move_skill_entry in self.move_list
        }

        return role_requests

    def tick(self, role_results: tactic.RoleResults) -> List[tactic.SkillEntry]:
        """
        :return: A list of size 1 skill depending on which role is filled
        """

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

