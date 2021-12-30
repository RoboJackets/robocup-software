import stp.play as play
import stp.tactic as tactic

from rj_gameplay.tactic import (
    wall_tactic,
    nmark_tactic,
    goalie_tactic,
    move_tactic,
)
import stp.skill as skill
import stp.role as role
from stp.role.assignment.naive import NaiveRoleAssignment
import stp.rc as rc
from typing import (
    Dict,
    Generic,
    Iterator,
    List,
    Optional,
    Tuple,
    Type,
    TypeVar,
)


class PreparePenaltyDefense(play.IPlay):
    """For when we don't have the ball and are trying to stop the opponent from scoring."""

    def __init__(self):
        self.goalie_move = move_tactic.Move((0.0, 1.0), face_point=(0.0, 4.5))
        self.goalie_move.cost = goalie_tactic.GoalieCost()
        self.tactics = [
            self.goalie_move,
            move_tactic.Move((1.6, 9.0)),
            move_tactic.Move((1.9, 9.0)),
            move_tactic.Move((2.2, 9.0)),
            move_tactic.Move((2.5, 9.0)),
            move_tactic.Move((2.8, 9.0)),
        ]

        self.role_assigner = NaiveRoleAssignment()

    def compute_props(self, prev_props):
        pass

    def tick(
        self,
        world_state: rc.WorldState,
        prev_results: role.assignment.FlatRoleResults,
        props,
    ) -> Tuple[
        Dict[Type[tactic.SkillEntry], List[role.RoleRequest]],
        List[tactic.SkillEntry],
    ]:

        # Get role requests from all tactics and put them into a dictionary
        role_requests: play.RoleRequests = {
            tactic: tactic.get_requests(world_state, None) for tactic in self.tactics
        }

        # Flatten requests and use role assigner on them
        flat_requests = play.flatten_requests(role_requests)
        flat_results = self.role_assigner.assign_roles(
            flat_requests, world_state, prev_results
        )
        role_results = play.unflatten_results(flat_results)

        # Get list of all SkillEntries from all tactics
        skills = []
        for tactic in self.tactics:
            skills += tactic.tick(world_state, role_results[tactic])

        # Get all role assignments
        # SkillEntry to (list of?) RoleResult
        skill_dict = {}
        for tactic in self.tactics:
            skill_dict.update(role_results[tactic])

        return (skill_dict, skills)

    def is_done(self, world_state):
        # returns done when all tactics are done
        # TODO: change all is_done() to use all()?
        return all([tactic.is_done(world_state) for tactic in self.tactics])


class PenaltyDefense(play.IPlay):
    """For when we don't have the ball and are trying to stop the opponent from scoring."""

    def __init__(self):
        self.tactics = [
            goalie_tactic.GoalieTactic(),
            move_tactic.Move((1.6, 9.0)),
            move_tactic.Move((1.9, 9.0)),
            move_tactic.Move((2.2, 9.0)),
            move_tactic.Move((2.5, 9.0)),
            move_tactic.Move((2.8, 9.0)),
        ]

        self.role_assigner = NaiveRoleAssignment()

    def compute_props(self, prev_props):
        pass

    def tick(
        self,
        world_state: rc.WorldState,
        prev_results: role.assignment.FlatRoleResults,
        props,
    ) -> Tuple[
        Dict[Type[tactic.SkillEntry], List[role.RoleRequest]],
        List[tactic.SkillEntry],
    ]:

        # Get role requests from all tactics and put them into a dictionary
        role_requests: play.RoleRequests = {
            tactic: tactic.get_requests(world_state, None) for tactic in self.tactics
        }

        # Flatten requests and use role assigner on them
        flat_requests = play.flatten_requests(role_requests)
        flat_results = self.role_assigner.assign_roles(
            flat_requests, world_state, prev_results
        )
        role_results = play.unflatten_results(flat_results)

        # Get list of all SkillEntries from all tactics
        skills = []
        for tactic in self.tactics:
            skills += tactic.tick(world_state, role_results[tactic])

        # Get all role assignments
        # SkillEntry to (list of?) RoleResult
        skill_dict = {}
        for tactic in self.tactics:
            skill_dict.update(role_results[tactic])

        return (skill_dict, skills)

    def is_done(self, world_state):
        # returns done when all tactics are done
        # TODO: change all is_done() to use all()?
        return all([tactic.is_done(world_state) for tactic in self.tactics])
