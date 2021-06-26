import stp.play as play
import stp.tactic as tactic

from rj_gameplay.tactic import clear_tactic, nmark_tactic, goalie_tactic, striker_tactic, move_tactic
import stp.skill as skill
import stp.role as role
from stp.role.assignment.naive import NaiveRoleAssignment
import stp.rc as rc
import numpy as np
from typing import Dict, Generic, Iterator, List, Optional, Tuple, Type, TypeVar

class KickCost(role.CostFn):
    """
    A cost function for how to choose a robot that will pass
    """

    def __call__(self, robot: rc.Robot, prev_result: Optional["RoleResult"],
                 world_state: rc.WorldState) -> float:
        if robot.id == 1:
            return 0
        else:
            return 99

class ShootCost(role.CostFn):
    """
    A cost function for how to choose a robot that will pass
    """

    def __call__(self, robot: rc.Robot, prev_result: Optional["RoleResult"],
                 world_state: rc.WorldState) -> float:
        if robot.id == 2:
            return 0
        else:
            return 99

class Attack(play.IPlay):

    def __init__(self):
        self.target_point = np.array([0.0, 5.0])
        self.shoot = striker_tactic.LineKickStrikerTactic(self.target_point)
        self.kick = striker_tactic.LineKick(target_point=self.target_point, kick_speed=1.0)
        self.kick.capture_cost = KickCost()
        self.shoot.capture_cost = ShootCost()
        self.role_assigner = NaiveRoleAssignment()
        self.shoot_now = False

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
        if not self.shoot_now:
            role_requests[self.kick] = self.kick.get_requests(world_state, None)
        else:
            role_requests[self.shoot] = self.shoot.get_requests(world_state, None)

        # Flatten requests and use role assigner on them
        flat_requests = play.flatten_requests(role_requests)
        flat_results = self.role_assigner.assign_roles(flat_requests, world_state, prev_results)
        role_results = play.unflatten_results(flat_results)

        # Get list of all skills with assigned roles from tactics
        # skills = self.two_mark.tick(role_results[self.two_mark])
        skills = []
        skill_dict = {}
        if not self.shoot_now:
            skills += self.kick.tick(role_results[self.kick], world_state)
            skill_dict.update(role_results[self.kick])
        else:
            skills += self.shoot.tick(role_results[self.shoot], world_state)
            skill_dict.update(role_results[self.shoot])

        if self.kick.is_done(world_state):
            self.shoot_now = True

        return (skill_dict, skills)

    def is_done(self, world_state):
        return self.shoot.is_done(world_state)
