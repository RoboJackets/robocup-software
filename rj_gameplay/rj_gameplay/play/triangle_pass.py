import stp.play as play
import stp.tactic as tactic

from rj_gameplay.tactic import pass_tactic, pass_seek
import stp.skill as skill
import stp.role as role
from stp.role.assignment.naive import NaiveRoleAssignment
import stp.rc as rc
from typing import Dict, Generic, Iterator, List, Optional, Tuple, Type, TypeVar
import numpy as np

class Passer1Cost(role.CostFn):
    """
    A cost function for how to choose a robot that will pass
    TODO: Implement a better cost function
    """
    def __init__(self):
        self.chosen_receiver = None

    def __call__(self,
                robot:rc.Robot,
                prev_result:Optional["RoleResult"],
                world_state:rc.WorldState) -> float:
        if robot.id == 1:
            return 0
        return 99

class Passer2Cost(role.CostFn):
    """
    A cost function for how to choose a robot that will pass
    TODO: Implement a better cost function
    """
    def __init__(self):
        self.chosen_receiver = None

    def __call__(self,
                robot:rc.Robot,
                prev_result:Optional["RoleResult"],
                world_state:rc.WorldState) -> float:
        if robot.id == 2:
            return 0
        return 99

class Passer3Cost(role.CostFn):
    """
    A cost function for how to choose a robot that will pass
    TODO: Implement a better cost function
    """
    def __init__(self):
        self.chosen_receiver = None

    def __call__(self,
                robot:rc.Robot,
                prev_result:Optional["RoleResult"],
                world_state:rc.WorldState) -> float:
        if robot.id == 3:
            return 0
        return 99

class TrianglePass(play.IPlay):
    """A play which makes one robot pass to another
    """

    def __init__(self):
        self.target_point = np.array([1.0,1.0])
        self.pass_tactic_1 = pass_tactic.Pass(
            self.target_point, Passer1Cost(),
            Passer2Cost())
        self.pass_tactic_2 = pass_tactic.Pass(
            self.target_point, Passer2Cost(),
            Passer3Cost())
        self.pass_tactic_3 = pass_tactic.Pass(
            self.target_point, Passer3Cost(),
            Passer1Cost())
        self.state = 1
        self.role_assigner = NaiveRoleAssignment()


    def compute_props(self, prev_props):
        pass

    def tick(
        self,
        world_state: rc.WorldState,
        prev_results: role.assignment.FlatRoleResults,
        props,
    ) -> Tuple[Dict[Type[tactic.SkillEntry], List[role.RoleRequest]], List[tactic.SkillEntry]]:
        # Get role requests from all tactics and put them into a dictionary
        role_requests: play.RoleRequests = {}
        if self.state == 1:
            role_requests[self.pass_tactic_1] = self.pass_tactic_1.get_requests(world_state, None)
        elif self.state == 2:
            role_requests[self.pass_tactic_2] = self.pass_tactic_2.get_requests(world_state, None)
        elif self.state ==3:
            role_requests[self.pass_tactic_3] = self.pass_tactic_3.get_requests(world_state, None)
        # Flatten requests and use role assigner on them
        flat_requests = play.flatten_requests(role_requests)
        flat_results = self.role_assigner.assign_roles(flat_requests, world_state, prev_results)
        role_results = play.unflatten_results(flat_results)

        # Get list of all skills with assigned roles from tactics
        skill_dict = {}
        skills = []
        if self.state == 1:
            skills = self.pass_tactic_1.tick(role_results[self.pass_tactic_1], world_state)
            skill_dict.update(role_results[self.pass_tactic_1])
        elif self.state == 2:
            skills = self.pass_tactic_2.tick(role_results[self.pass_tactic_2], world_state)
            skill_dict.update(role_results[self.pass_tactic_2])
        elif self.state == 3:
            skills = self.pass_tactic_3.tick(role_results[self.pass_tactic_3], world_state)
            skill_dict.update(role_results[self.pass_tactic_3])

        if self.state == 1 and self.pass_tactic_1.is_done(world_state):
            self.state = 2
            self.pass_tactic_1 = pass_tactic.Pass(self.target_point, Passer1Cost(),Passer2Cost())
        elif self.state == 2 and self.pass_tactic_2.is_done(world_state):
            self.state = 3
            self.pass_tactic_2 = pass_tactic.Pass(self.target_point, Passer2Cost(),Passer3Cost())
        elif self.state == 3 and self.pass_tactic_3.is_done(world_state):
            self.state = 1
            self.pass_tactic_3 = pass_tactic.Pass(self.target_point, Passer3Cost(),Passer1Cost())
        print(self.state)

        return (skill_dict, skills)

    def is_done(self, world_state: rc.WorldState):
        return False
