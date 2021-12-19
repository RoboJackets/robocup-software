import stp.play as play
import stp.tactic as tactic

from rj_gameplay.tactic import pass_tactic, pass_seek
import stp.skill as skill
import stp.role as role
from stp.role.assignment.naive import NaiveRoleAssignment
import stp.rc as rc
from typing import Dict, Generic, Iterator, List, Optional, Tuple, Type, TypeVar
import numpy as np

class PassPlay(play.IPlay):
    """A play which makes one robot pass to another
    """

    def __init__(self, action_client_dict: Dict[Type[Any], List[Any]]):
        self.target_point = np.array([1.0,1.0])
        self.pass_tactic = pass_tactic.Pass(
            self.target_point, pass_tactic.PasserCost(),
            pass_tactic.PassToOpenReceiver(self.target_point))
        self.seek_tactic = pass_seek.Seek(
            self.target_point, pass_seek.seek_heuristic,
            pass_seek.SeekCost(self.target_point))
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
        role_requests[self.pass_tactic] = self.pass_tactic.get_requests(
            world_state, None)
        if not self.pass_tactic.pivot_kick.skill.is_done(world_state):
            role_requests[self.seek_tactic] = self.seek_tactic.get_requests(
                world_state, None)
        # Flatten requests and use role assigner on them
        flat_requests = play.flatten_requests(role_requests)
        flat_results = self.role_assigner.assign_roles(flat_requests, world_state, prev_results)
        role_results = play.unflatten_results(flat_results)

        # Get list of all skills with assigned roles from tactics
        skill_dict = {}
        skills = []
        skills = self.pass_tactic.tick(world_state,
                                       role_results[self.pass_tactic])
        skill_dict.update(role_results[self.pass_tactic])
        if not self.pass_tactic.pivot_kick.skill.is_done(world_state):
            skills += self.seek_tactic.tick(world_state,
                                            role_results[self.seek_tactic])
            skill_dict.update(role_results[self.seek_tactic])

        return (skill_dict, skills)

    def is_done(self, world_state: rc.WorldState):
        return self.pass_tactic.is_done(world_state)
