import stp.play as play
import stp.tactic as tactic

from rj_gameplay.tactic import move_tactic
import stp.skill as skill
import stp.role as role
from stp.role.assignment.naive import NaiveRoleAssignment
import stp.rc as rc
from typing import Dict, Generic, Iterator, List, Optional, Tuple, Type, TypeVar
import numpy as np

class LineUp(play.IPlay):
    """A play which lines up two robots, one on the right the one on the left
    """

    def __init__(self):
        self.left_x = 1.0
        self.right_x = -1.5
        self.start_y = 2.0
        self.y_inc = 0.3
        self.move_right = move_tactic.Move(np.array([self.right_x, self.start_y]))
        self.move_left = move_tactic.Move(np.array([self.left_x, self.start_y]))
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
        if self.move_right.is_done(world_state):
            role_requests[self.move_left] = self.move_left.get_requests(world_state, None)
        else:
            role_requests[self.move_right] = self.move_right.get_requests(world_state, None)
        # Flatten requests and use role assigner on them
        flat_requests = play.flatten_requests(role_requests)
        flat_results = self.role_assigner.assign_roles(flat_requests, world_state, prev_results)
        role_results = play.unflatten_results(flat_results)

        # Get list of all skills with assigned roles from tactics
        skill_dict = {}
        if self.move_right.is_done(world_state):
            skills = self.move_left.tick(role_results[self.move_left])
            skill_dict.update(role_results[self.move_left])
        else:
            skills = self.move_right.tick(role_results[self.move_right])
            skill_dict.update(role_results[self.move_right])
        # skills = self.move_right.tick(role_results[self.move_right]) + self.move_left.tick(role_results[self.move_left])
        # skill_dict = {}
        # skill_dict.update(role_results[self.move_right])
        # skill_dict.update(role_results[self.move_left])

        return (skill_dict ,skills)

    def is_done(self ,world_state):
        return self.move_left.is_done(world_state)
