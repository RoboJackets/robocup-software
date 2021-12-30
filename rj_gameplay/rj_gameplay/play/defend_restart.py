import stp.play as play
import stp.tactic as tactic

from rj_gameplay.tactic import nmark_tactic, goalie_tactic, wall_tactic
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
from rj_gameplay.calculations import wall_calculations


class DefendRestart(play.IPlay):
    def __init__(self):
        # TODO: add chipper tactic here
        self.goalie = goalie_tactic.GoalieTactic()
        self.markers = nmark_tactic.NMarkTactic(3)
        self.wall_1 = wall_tactic.WallTactic()
        self.wall_2 = wall_tactic.WallTactic()
        self.role_assigner = NaiveRoleAssignment()

        self.num_wallers = 2

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

        # pre-calculate wall points and store in numpy array
        wall_pts = wall_calculations.find_wall_pts(
            self.num_wallers, world_state
        )

        # Get role requests from all tactics and put them into a dictionary
        role_requests: play.RoleRequests = {}
        role_requests[self.markers] = self.markers.get_requests(
            world_state, None
        )
        role_requests[self.goalie] = self.goalie.get_requests(
            world_state, None
        )
        role_requests[self.wall_1] = self.wall_1.get_requests(
            world_state, wall_pts[0], None
        )
        role_requests[self.wall_2] = self.wall_2.get_requests(
            world_state, wall_pts[1], None
        )

        # Flatten requests and use role assigner on them
        flat_requests = play.flatten_requests(role_requests)
        flat_results = self.role_assigner.assign_roles(
            flat_requests, world_state, prev_results
        )
        role_results = play.unflatten_results(flat_results)

        # Get list of all skills with assigned roles from tactics

        skills = self.markers.tick(world_state, role_results[self.markers])
        skills += self.goalie.tick(world_state, role_results[self.goalie])
        skills += self.wall_1.tick(world_state, role_results[self.wall_1])
        skills += self.wall_2.tick(world_state, role_results[self.wall_2])
        skill_dict = {}
        skill_dict.update(role_results[self.markers])
        skill_dict.update(role_results[self.goalie])
        skill_dict.update(role_results[self.wall_1])
        skill_dict.update(role_results[self.wall_2])

        return (skill_dict, skills)

    def is_done(self, world_state):
        return self.markers.is_done(world_state)
