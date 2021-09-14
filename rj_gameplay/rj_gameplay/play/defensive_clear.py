import stp.play as play
import stp.tactic as tactic

from rj_gameplay.tactic import clear_tactic, nmark_tactic, goalie_tactic
import stp.skill as skill
import stp.role as role
from stp.role.assignment.naive import NaiveRoleAssignment
import stp.rc as rc
import numpy as np
from typing import Dict, Generic, Iterator, List, Optional, Tuple, Type, TypeVar

class DefensiveClear(play.IPlay):

    def __init__(self):
        self.goalie = goalie_tactic.GoalieTactic()
        self.two_mark = nmark_tactic.NMarkTactic(2)
        self.clear = clear_tactic.Clear(np.array([0.0, 10.0]))
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
        role_requests[self.two_mark] = (self.two_mark.get_requests(
            world_state, None))
        role_requests[self.clear] = self.clear.get_requests(world_state, None)
        role_requests[self.goalie] = self.goalie.get_requests(
            world_state, None)

        # Flatten requests and use role assigner on them
        flat_requests = play.flatten_requests(role_requests)
        flat_results = self.role_assigner.assign_roles(flat_requests,
                                                       world_state,
                                                       prev_results)
        role_results = play.unflatten_results(flat_results)

        # Get list of all skills with assigned roles from tactics
        skills = self.two_mark.tick(world_state, role_results[self.two_mark])
        skills += self.clear.tick(world_state, role_results[self.clear])
        skills += self.goalie.tick(world_state, role_results[self.goalie])
        skill_dict = {}
        skill_dict.update(role_results[self.two_mark])
        skill_dict.update(role_results[self.clear])
        skill_dict.update(role_results[self.goalie])

        return (skill_dict, skills)

    def is_done(self, world_state):
        return self.clear.is_done(world_state)
