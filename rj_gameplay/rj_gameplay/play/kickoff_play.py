import stp.play as play
import stp.tactic as tactic

from rj_gameplay.tactic import move_tactic, goalie_tactic, wall_tactic
import stp.skill as skill
import stp.role as role
from stp.role.assignment.naive import NaiveRoleAssignment
import stp.rc as rc
from typing import Dict, Generic, Iterator, List, Optional, Tuple, Type, TypeVar
import numpy as np


class kickoff_cost(role.CostFn):
    def __call__(self, robot: rc.Robot, prev_result: Optional["RoleResult"],
                 world_state: rc.WorldState) -> float:
        return 0.0


class PrepareKickoffPlay(play.IPlay):
    """Hardcoded points to stand in for kickoff
    """

    def __init__(self):
        self.points = [
                (0.0, 4.25),
                (0.6, 4.25),
                (-0.6, 4.25),
            ]
        self.tactics = [move_tactic.Move(target_point=np.array(pt), face_point=(0.0, 4.5)) for pt in self.points]
        self.tactics.append(goalie_tactic.GoalieTactic())
        self.tactics.append(wall_tactic.WallTactic(2))

        # self.move_left = move_tactic.Move(np.array([self.left_x, self.start_y]))
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
        role_requests: play.RoleRequests = {
            tactic: tactic.get_requests(world_state, None)
            for tactic in self.tactics
        }

        # Flatten requests and use role assigner on them
        flat_requests = play.flatten_requests(role_requests)
        flat_results = self.role_assigner.assign_roles(flat_requests,
                                                       world_state,
                                                       prev_results)
        role_results = play.unflatten_results(flat_results)

        # Get list of all SkillEntries from all tactics
        skills = []
        for tactic in self.tactics:
            skills += tactic.tick(role_results[tactic])

        # Get all role assignments
        # SkillEntry to (list of?) RoleResult
        skill_dict = {}
        for tactic in self.tactics:
            skill_dict.update(role_results[tactic])

        return (skill_dict ,skills)

    def is_done(self ,world_state):
        # last tactic done (HACK)
        return self.tactics[-1].is_done(world_state)

class DefendKickoffPlay(play.IPlay):
    """Hardcoded points to stand in for kickoff
    """

    def __init__(self):
        self.points = [
            (0.0, 3.9),
            (0.8, 4.25),
            (-0.8, 4.25),
        ]
        self.tactics = [move_tactic.Move(target_point=np.array(pt), face_point=(0.0, 4.5)) for pt in self.points]
        self.tactics.append(goalie_tactic.GoalieTactic())
        self.tactics.append(wall_tactic.WallTactic(2))

        # self.move_left = move_tactic.Move(np.array([self.left_x, self.start_y]))
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
        role_requests: play.RoleRequests = {
            tactic: tactic.get_requests(world_state, None)
            for tactic in self.tactics
        }

        # Flatten requests and use role assigner on them
        flat_requests = play.flatten_requests(role_requests)
        flat_results = self.role_assigner.assign_roles(flat_requests,
                                                       world_state,
                                                       prev_results)
        role_results = play.unflatten_results(flat_results)

        # Get list of all SkillEntries from all tactics
        skills = []
        for tactic in self.tactics:
            skills += tactic.tick(role_results[tactic])

        # Get all role assignments
        # SkillEntry to (list of?) RoleResult
        skill_dict = {}
        for tactic in self.tactics:
            skill_dict.update(role_results[tactic])

        return (skill_dict ,skills)

    def is_done(self ,world_state):
        # last tactic done (HACK)
        return self.tactics[-1].is_done(world_state)
