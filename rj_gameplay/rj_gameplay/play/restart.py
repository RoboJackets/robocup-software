import stp.play as play
import stp.tactic as tactic

from rj_gameplay.tactic import pass_tactic, pass_seek, nmark_tactic, goalie_tactic, clear_tactic, wall_tactic
import stp.skill as skill
import stp.role as role
from stp.role.assignment.naive import NaiveRoleAssignment
import stp.rc as rc
from typing import Dict, Generic, Iterator, List, Optional, Tuple, Type, TypeVar
import numpy as np
from rj_gameplay.calculations import wall_calculations


class RestartPlay(play.IPlay):
    """One robot passes to another. Some markers.
    """
    def __init__(self, action_client_dict: Dict[Type[Any], List[Any]]):
        self.target_point = np.array([1.0, 4.0])

        # TODO: simplify tactic with list (see basic_defense.py)
        self.goalie_tactic = goalie_tactic.GoalieTactic()
        self.clear_tactic = clear_tactic.Clear(np.array([0.0, 9.0]),
                                               chip=True,
                                               kick_speed=3.0)
        # TODO: make it pass
        """
        self.pass_tactic = pass_tactic.Pass(
            self.target_point, pass_tactic.PasserCost(self.target_point),
            pass_tactic.PassToClosestReceiver(self.target_point))
        self.seek_tactic = pass_seek.Seek(
            self.target_point, pass_seek.restart_seek,
            pass_seek.SeekCost(self.target_point))
        """
        self.wall_tactic_1 = wall_tactic.WallTactic(role.Priority.LOW,
                                                    cost_scale=0.1)
        self.wall_tactic_2 = wall_tactic.WallTactic(role.Priority.LOW,
                                                    cost_scale=0.1)

        left_pt = np.array([1.5, 7.5])
        self.seek_left = pass_seek.Seek(left_pt,
                                        pass_seek.build_seek_function(left_pt),
                                        pass_seek.SeekCost(left_pt))

        right_pt = np.array([-1.5, 7.5])
        self.seek_right = pass_seek.Seek(
            right_pt, pass_seek.build_seek_function(right_pt),
            pass_seek.SeekCost(right_pt))

        self.role_assigner = NaiveRoleAssignment()

        # number of wallers for finding wall_pts
        self.num_wallers = 2

    def compute_props(self, prev_props):
        pass

    def tick(
        self,
        world_state: rc.WorldState,
        prev_results: role.assignment.FlatRoleResults,
        props,
    ) -> Tuple[Dict[Type[tactic.SkillEntry], List[role.RoleRequest]],
               List[tactic.SkillEntry]]:

        # pre-calculate wall points and store in numpy array
        wall_pts = wall_calculations.find_wall_pts(self.num_wallers,
                                                   world_state)

        # Get role requests from all tactics and put them into a dictionary
        role_requests: play.RoleRequests = {}
        # role_requests[self.pass_tactic] = self.pass_tactic.get_requests(world_state, None)
        # role_requests[self.seek_tactic] = self.seek_tactic.get_requests(world_state, None)
        role_requests[self.clear_tactic] = self.clear_tactic.get_requests(
            world_state, None)
        role_requests[self.wall_tactic_1] = self.wall_tactic_1.get_requests(
            world_state, wall_pts[0], None)
        role_requests[self.wall_tactic_2] = self.wall_tactic_2.get_requests(
            world_state, wall_pts[1], None)
        role_requests[self.goalie_tactic] = self.goalie_tactic.get_requests(
            world_state, None)
        role_requests[self.seek_left] = self.seek_left.get_requests(
            world_state, None)
        role_requests[self.seek_right] = self.seek_right.get_requests(
            world_state, None)

        # Flatten requests and use role assigner on them
        flat_requests = play.flatten_requests(role_requests)
        flat_results = self.role_assigner.assign_roles(flat_requests,
                                                       world_state,
                                                       prev_results)
        role_results = play.unflatten_results(flat_results)

        # Get list of all skills with assigned roles from tactics
        skill_dict = {}
        skills = []
        skills = self.clear_tactic.tick(world_state,
                                        role_results[self.clear_tactic])
        skills += self.goalie_tactic.tick(world_state,
                                          role_results[self.goalie_tactic])
        skills += self.wall_tactic_1.tick(world_state,
                                          role_results[self.wall_tactic_1])
        skills += self.wall_tactic_2.tick(world_state,
                                          role_results[self.wall_tactic_2])
        skills += self.seek_left.tick(world_state,
                                      role_results[self.seek_left])
        skills += self.seek_right.tick(world_state,
                                       role_results[self.seek_right])
        skill_dict.update(role_results[self.clear_tactic])
        skill_dict.update(role_results[self.goalie_tactic])
        skill_dict.update(role_results[self.wall_tactic_1])
        skill_dict.update(role_results[self.wall_tactic_2])
        skill_dict.update(role_results[self.seek_left])
        skill_dict.update(role_results[self.seek_right])

        return (skill_dict, skills)

    def is_done(self, world_state: rc.WorldState):
        return self.clear_tactic.is_done(world_state)


class DirectRestartPlay(play.IPlay):
    """One robot passes to another. Some markers.
    """
    def __init__(self, action_client_dict: Dict[Type[Any], List[Any]]):
        self.target_point = np.array([1.0, 4.0])

        # TODO: simplify tactic with list (see basic_defense.py)
        self.goalie_tactic = goalie_tactic.GoalieTactic()
        self.clear_tactic = clear_tactic.Clear(np.array([0.0, 9.0]),
                                               chip=False,
                                               kick_speed=5.5)
        # TODO: make it pass
        """
        self.pass_tactic = pass_tactic.Pass(
            self.target_point, pass_tactic.PasserCost(self.target_point),
            pass_tactic.PassToClosestReceiver(self.target_point))
        self.seek_tactic = pass_seek.Seek(
            self.target_point, pass_seek.restart_seek,
            pass_seek.SeekCost(self.target_point))
        """
        self.wall_tactic_1 = wall_tactic.WallTactic(role.Priority.LOW,
                                                    cost_scale=0.1)
        self.wall_tactic_2 = wall_tactic.WallTactic(role.Priority.LOW,
                                                    cost_scale=0.1)

        # might need to change to for-loop
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
    ) -> Tuple[Dict[Type[tactic.SkillEntry], List[role.RoleRequest]],
               List[tactic.SkillEntry]]:

        # pre-calculate wall points and store in numpy array
        wall_pts = calculations.find_wall_pts(self.num_wallers, world_state)

        # Get role requests from all tactics and put them into a dictionary
        role_requests: play.RoleRequests = {}
        # role_requests[self.pass_tactic] = self.pass_tactic.get_requests(world_state, None)
        # role_requests[self.seek_tactic] = self.seek_tactic.get_requests(world_state, None)
        role_requests[self.clear_tactic] = self.clear_tactic.get_requests(
            world_state, None)
        role_requests[self.wall_tactic_1] = self.wall_tactic_1.get_requests(
            world_state, wall_pts[0], None)
        role_requests[self.wall_tactic_2] = self.wall_tactic_2.get_requests(
            world_state, wall_pts[1], None)
        role_requests[self.goalie_tactic] = self.goalie_tactic.get_requests(
            world_state, None)
        role_requests[self.seek_left] = self.seek_left.get_requests(
            world_state, None)
        role_requests[self.seek_right] = self.seek_right.get_requests(
            world_state, None)

        # Flatten requests and use role assigner on them
        flat_requests = play.flatten_requests(role_requests)
        flat_results = self.role_assigner.assign_roles(flat_requests,
                                                       world_state,
                                                       prev_results)
        role_results = play.unflatten_results(flat_results)

        # Get list of all skills with assigned roles from tactics
        skill_dict = {}
        skills = []
        skills = self.clear_tactic.tick(world_state,
                                        role_results[self.clear_tactic])
        skills += self.goalie_tactic.tick(world_state,
                                          role_results[self.goalie_tactic])
        skills += self.wall_tactic_1.tick(world_state,
                                          role_results[self.wall_tactic_1])
        skills += self.wall_tactic_2.tick(world_state,
                                          role_results[self.wall_tactic_2])
        skills += self.seek_left.tick(world_state,
                                      role_results[self.seek_left])
        skills += self.seek_right.tick(world_state,
                                       role_results[self.seek_right])
        skill_dict.update(role_results[self.clear_tactic])
        skill_dict.update(role_results[self.goalie_tactic])
        skill_dict.update(role_results[self.wall_tactic_1])
        skill_dict.update(role_results[self.wall_tactic_2])
        skill_dict.update(role_results[self.seek_left])
        skill_dict.update(role_results[self.seek_right])

        return (skill_dict, skills)

    def is_done(self, world_state: rc.WorldState):
        return self.clear_tactic.is_done(world_state)
