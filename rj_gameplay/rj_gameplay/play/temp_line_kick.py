import stp.play as play
import stp.tactic as tactic

from rj_gameplay.tactic import temp_line_kick 
import stp.skill as skill
import stp.role as role
from stp.role.assignment.naive import NaiveRoleAssignment
import stp.rc as rc
from typing import Dict, Generic, Iterator, List, Optional, Tuple, Type, TypeVar
import numpy as np

class LineKickPlay(play.IPlay):
    """
    Dummy play to kick a stationary ball, to test the line_kick skill.
    """

    def __init__(self, world_state: rc.WorldState):
        """
        self.left_x = 1.0
        self.right_x = -1.5
        self.start_y = 2.0
        self.y_inc = 0.3
        self.move_right = move_tactic.Move(np.array([self.right_x, self.start_y]))
        self.move_left = move_tactic.Move(np.array([self.left_x, self.start_y]))
        """
        self.kicker = temp_line_kick.LineKickTactic(world_state)
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
        role_requests[self.kicker] = self.kicker.get_requests(world_state, None)

        # Flatten requests and use role assigner on them, then unflatten
        flat_requests = play.flatten_requests(role_requests)
        flat_results = self.role_assigner.assign_roles(flat_requests, world_state, prev_results)
        role_results = play.unflatten_results(flat_results)

        # Get list of all skills with assigned roles from tactics
        skill_dict = {}
        skills = self.kicker.tick(role_results[self.kicker])
        skill_dict.update(role_results[self.kicker])

        return (skill_dict ,skills)

    def is_done(self ,world_state):
        return self.kicker.is_done(world_state)
