import stp.play as play
import stp.tactic as tactic

from rj_gameplay.tactic import striker_tactic, nmark_tactic, goalie_tactic, pass_seek, wall_tactic
import stp.skill as skill
import stp.role as role
from stp.role.assignment.naive import NaiveRoleAssignment
import stp.rc as rc
from typing import Dict, Generic, Iterator, List, Optional, Tuple, Type, TypeVar
import numpy as np
from rj_gameplay.calculations import wall_calculations


class Basic122(play.IPlay):
    """A basic offensive play. One robot is a striker and shoots as soon it gets the ball.
    other two seek to specific spots (BAD HARDCODING)
    """

    def __init__(self):
        self.target_point: np.ndarray = np.array([0., 9.])
        self.striker_tactic = striker_tactic.LineKickStrikerTactic(
            target_point=self.target_point)
        self.goalie_tactic = goalie_tactic.GoalieTactic()
        self.wall_tactic_1 = wall_tactic.WallTactic(role.Priority.LOW,
                                                    cost_scale=0.1)
        self.wall_tactic_2 = wall_tactic.WallTactic(role.Priority.LOW,
                                                    cost_scale=0.1)

        self.num_wallers = 2

        left_pt = np.array([1.5, 7.5])
        self.seek_left = pass_seek.Seek(left_pt,
                                        pass_seek.build_seek_function(left_pt),
                                        pass_seek.SeekCost(left_pt))

        right_pt = np.array([-1.5, 7.5])
        self.seek_right = pass_seek.Seek(
            right_pt, pass_seek.build_seek_function(right_pt),
            pass_seek.SeekCost(right_pt))

        self.role_assigner = NaiveRoleAssignment()

    def compute_props(self, prev_props):
        pass

    def tick(
        self,
        world_state: rc.WorldState,
        prev_results: role.assignment.FlatRoleResults,
        props,
    ) -> Tuple[Dict[tactic.SkillEntry,
                    List[role.RoleResult]], List[tactic.SkillEntry]]:
        # TODO: create NamedTuple for this type
        # pre-calculate wall points and store in numpy array
        wall_pts = wall_calculations.find_wall_pts(self.num_wallers,
                                                   world_state)

        # Get role requests from all tactics and put them into a dictionary

        role_requests: play.RoleRequests = {
            self.striker_tactic:
            self.striker_tactic.get_requests(world_state, None),
            # self.two_mark: self.two_mark.get_requests(world_state, None),
            self.seek_left:
            self.seek_left.get_requests(world_state, None),
            self.seek_right:
            self.seek_right.get_requests(world_state, None),
            self.goalie_tactic:
            self.goalie_tactic.get_requests(world_state, None),
            self.wall_tactic_1:
            self.wall_tactic_1.get_requests(world_state, wall_pts[0], None),
            self.wall_tactic_2:
            self.wall_tactic_2.get_requests(world_state, wall_pts[1], None)
        }

        # Flatten requests and use role assigner on them
        flat_requests = play.flatten_requests(role_requests)
        flat_results = self.role_assigner.assign_roles(flat_requests,
                                                       world_state,
                                                       prev_results)
        role_results = play.unflatten_results(flat_results)

        # Get list of all skills with assigned roles from tactics

        skills = self.striker_tactic.tick(world_state,
                                          role_results[self.striker_tactic])
        skills += self.seek_left.tick(world_state,
                                      role_results[self.seek_left])
        skills += self.seek_right.tick(world_state,
                                       role_results[self.seek_right])
        skills += self.goalie_tactic.tick(world_state,
                                          role_results[self.goalie_tactic])
        skills += self.wall_tactic_1.tick(world_state,
                                          role_results[self.wall_tactic_1])
        skills += self.wall_tactic_2.tick(world_state,
                                          role_results[self.wall_tactic_2])

        skill_dict = {}
        skill_dict.update(role_results[self.striker_tactic])
        skill_dict.update(role_results[self.seek_left])
        skill_dict.update(role_results[self.seek_right])
        skill_dict.update(role_results[self.goalie_tactic])
        skill_dict.update(role_results[self.wall_tactic_1])
        skill_dict.update(role_results[self.wall_tactic_2])
        return skill_dict, skills

    def is_done(self, world_state):
        return self.striker_tactic.is_done(world_state)
