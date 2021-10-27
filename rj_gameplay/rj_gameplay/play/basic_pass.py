import stp.play as play
import stp.tactic as tactic

from rj_gameplay.tactic import pass_tactic, pass_seek, nmark_tactic, goalie_tactic
import stp.role as role
from stp.role.assignment.naive import NaiveRoleAssignment
from typing import Dict, List, Tuple, Type
import stp.rc as rc
import numpy as np


class BasicPass(play.IPlay):
    """
    Basic Passing
    Receiver switches from seek tactic to pass tactic 
    """
    def __init__(self):
        self.tactics = [
            pass_tactic.Pass(np.array([0., 0.]), pass_tactic.PasserCost(),
                             pass_tactic.PassToBestReceiver()),
            pass_tactic.Pass(np.array([0., 0.]), pass_tactic.PasserCost(),
                             pass_tactic.PassToBestReceiver()),
            pass_seek.Seek(np.array([0., 0.]),
                           pass_seek.build_seek_function(np.array([0., 0.])),
                           pass_seek.SeekCost(np.array([0., 0.]))),
            pass_seek.Seek(np.array([0., 0.]),
                           pass_seek.build_seek_function(np.array([0., 0.])),
                           pass_seek.SeekCost(np.array([0., 0.]))),
            pass_seek.Seek(np.array([0., 0.]),
                           pass_seek.build_seek_function(np.array([0., 0.])),
                           pass_seek.SeekCost(np.array([0., 0.]))),
            nmark_tactic.NMarkTactic(1),
            goalie_tactic.GoalieTactic()
        ]

        self.role_assigner = NaiveRoleAssignment()

    def compute_props(self, prev_props):
        pass

    def tick(
        self,
        world_state: rc.WorldState,
        prev_results: role.assignment.FlatRoleResults,
        props,
    ) -> Tuple[Dict[Type[tactic.SkillEntry], List[role.RoleRequest]],
               List[tactic.SkillEntry]]:

        role_requests: play.RoleRequests = {}
        for tactic in self.tactics:
            role_requests[tactic] = tactic.get_requests(world_state, None)

        flat_requests = play.flatten_requests(role_requests)
        flat_results = self.role_assigner.assign_roles(flat_requests,
                                                       world_state,
                                                       prev_results)
        role_results = play.unflatten_results(flat_results)

        skills = []
        for tactic in self.tactics:
            skills += tactic.tick(world_state, role_results[tactic])

        skill_dict = {}
        for tactic in self.tactics:
            skill_dict.update(role_results[tactic])

        return skill_dict, skills

    def is_done(self, world_state):

        return False
