import stp.play as play
import stp.tactic as tactic

from rj_gameplay.tactic import striker_tactic, nmark_tactic, goalie_tactic, pass_seek, pass_tactic, assist_tactic
import stp.skill as skill
import stp.role as role
from stp.role.assignment.naive import NaiveRoleAssignment
import stp.rc as rc
from typing import Dict, Generic, Iterator, List, Optional, Tuple, Type, TypeVar
import numpy as np

def shoot_cost(world_state: rc.WorldState) -> bool:
    for bot in world_state.our_robots:
        if bot.has_ball_sense:
            robot = bot
    shoot_cost = assist_tactic.find_striker_cost(robot, world_state)
    for bot in world_state.our_robots:
        if bot.id != robot.id and assist_tactic.find_striker_cost(bot, world_state) < shoot_cost:
            return False
    return True

class PassOrShoot(play.IPlay):
    """A basic offensive play. One robot is a striker and shoots as soon it gets the ball.
    other two seek to specific spots (BAD HARDCODING)
    """

    def __init__(self):
        self.target_point: np.ndarray = np.array([0., 9.])
        self.striker_tactic = striker_tactic.StrikerTactic(target_point=self.target_point)
        self.pass_tactic = pass_tactic.Pass(
            self.target_point, pass_tactic.PasserCost(),
            pass_tactic.PassToOpenReceiver(self.target_point))
        self.goalie_tactic = goalie_tactic.GoalieTactic()
        self.shoot = None
        # self.two_mark = nmark_tactic.NMarkTactic(2)

        left_pt = np.array([1.5, 7.5])
        self.seek_left = pass_seek.Seek(left_pt, pass_seek.build_seek_function(left_pt), pass_seek.SeekCost(left_pt))

        right_pt = np.array([-1.5, 7.5])
        self.seek_right = pass_seek.Seek(right_pt, pass_seek.build_seek_function(right_pt), pass_seek.SeekCost(right_pt))

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

        role_requests = {}

        if self.striker_tactic.is_done(world_state):
            self.striker_tactic = striker_tactic.StrikerTactic(target_point=self.target_point)
        if self.pass_tactic.is_done(world_state):
            self.pass_tactic = pass_tactic.Pass(
                self.target_point, pass_tactic.PasserCost(),
                pass_tactic.PassToOpenReceiver(self.target_point))

        if not self.striker_tactic.capture.skill.is_done(world_state) and self.shoot is None:
            role_requests: play.RoleRequests = {self.striker_tactic: self.striker_tactic.get_requests(world_state, None),
                                                # self.two_mark: self.two_mark.get_requests(world_state, None),
                                                self.seek_left: self.seek_left.get_requests(world_state, None),
                                                self.seek_right: self.seek_right.get_requests(world_state, None),
                                                self.goalie_tactic: self.goalie_tactic.get_requests(world_state, None)}
        elif self.striker_tactic.capture.skill.is_done(world_state) or self.pass_tactic.pivot_kick.skill.is_done(world_state):
            if self.shoot is None:
                self.shoot = shoot_cost(world_state)
            if self.shoot:
                role_requests: play.RoleRequests = {self.striker_tactic: self.striker_tactic.get_requests(world_state, None),
                                                    # self.two_mark: self.two_mark.get_requests(world_state, None),
                                                    self.seek_left: self.seek_left.get_requests(world_state, None),
                                                    self.seek_right: self.seek_right.get_requests(world_state, None),
                                                    self.goalie_tactic: self.goalie_tactic.get_requests(world_state, None)}
            else:
                role_requests: play.RoleRequests = {self.pass_tactic: self.pass_tactic.get_requests(world_state, None),
                                                    # self.two_mark: self.two_mark.get_requests(world_state, None),
                                                    self.seek_left: self.seek_left.get_requests(world_state, None),
                                                    self.seek_right: self.seek_right.get_requests(world_state, None),
                                                    self.goalie_tactic: self.goalie_tactic.get_requests(world_state, None)}


        # Flatten requests and use role assigner on them
        flat_requests = play.flatten_requests(role_requests)
        flat_results = self.role_assigner.assign_roles(flat_requests, world_state, prev_results)
        role_results = play.unflatten_results(flat_results)

        skill_dict = {}

        # Get list of all skills with assigned roles from tactics
        skills = []
        if not self.striker_tactic.capture.skill.is_done(world_state) and self.shoot is None:
            skills = self.striker_tactic.tick(role_results[self.striker_tactic], world_state)
            skill_dict.update(role_results[self.striker_tactic])
        elif self.striker_tactic.capture.skill.is_done(world_state) or self.pass_tactic.pivot_kick.skill.is_done(world_state):
            if self.shoot:
                skills = self.striker_tactic.tick(role_results[self.striker_tactic], world_state)
                skill_dict.update(role_results[self.striker_tactic])
            else:
                skills = self.pass_tactic.tick(role_results[self.pass_tactic], world_state)
                skill_dict.update(role_results[self.pass_tactic])

        # skills += self.two_mark.tick(role_results[self.two_mark])
        skills += self.seek_left.tick(role_results[self.seek_left], world_state)
        skills += self.seek_right.tick(role_results[self.seek_right], world_state)
        skills += self.goalie_tactic.tick(role_results[self.goalie_tactic])

        
        # skill_dict.update(role_results[self.two_mark])
        skill_dict.update(role_results[self.seek_left])
        skill_dict.update(role_results[self.seek_right])
        skill_dict.update(role_results[self.goalie_tactic])

        return skill_dict, skills

    def is_done(self, world_state):
        print(self.striker_tactic.is_done(world_state) or self.pass_tactic.is_done(world_state))
        return self.striker_tactic.is_done(world_state) or self.pass_tactic.is_done(world_state)
