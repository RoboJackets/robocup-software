import stp.play as play
import stp.tactic as tactic

from rj_gameplay.tactic import striker_tactic, nmark_tactic, goalie_tactic
import stp.skill as skill
import stp.role as role
from stp.role.assignment.naive import NaiveRoleAssignment
import stp.rc as rc
from typing import Dict, Generic, Iterator, List, Optional, Tuple, Type, TypeVar
import numpy as np


class Basic122(play.IPlay):
    """A basic offensive play. One robot is a striker and shoots as soon it gets the ball.
    The othe two robots mark based on some heuristic
    """

    def __init__(self):
        self.target_point: np.ndarray = np.array([0., 12.])
        self.striker_tactic = striker_tactic.StrikerTactic(target_point=self.target_point)
        self.goalie_tactic = goalie_tactic.GoalieTactic()
        self.two_mark = nmark_tactic.NMarkTactic(2)
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

        role_requests: play.RoleRequests = {self.striker_tactic: self.striker_tactic.get_requests(world_state, None),
                                            self.two_mark: self.two_mark.get_requests(world_state, None),
                                            self.goalie_tactic: self.goalie_tactic.get_requests(world_state, None)}

        # Flatten requests and use role assigner on them
        flat_requests = play.flatten_requests(role_requests)
        flat_results = self.role_assigner.assign_roles(flat_requests, world_state, prev_results)
        role_results = play.unflatten_results(flat_results)

        # Get list of all skills with assigned roles from tactics

        skills = self.striker_tactic.tick(role_results[self.striker_tactic], world_state)
        skills += self.two_mark.tick(role_results[self.two_mark])
        skills += self.goalie_tactic.tick(role_results[self.goalie_tactic])

        skill_dict = {}
        skill_dict.update(role_results[self.striker_tactic])
        skill_dict.update(role_results[self.two_mark])
        skill_dict.update(role_results[self.goalie_tactic])
        return skill_dict, skills

    def is_done(self, world_state):
        return self.two_mark.is_done(world_state) and self.striker_tactic.is_done(world_state)
