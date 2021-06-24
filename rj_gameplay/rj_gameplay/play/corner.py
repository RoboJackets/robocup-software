import stp.play as play
import stp.tactic as tactic

from rj_gameplay.tactic import clear_tactic, nmark_tactic, goalie_tactic, striker_tactic, move_tactic
import stp.skill as skill
import stp.role as role
from stp.role.assignment.naive import NaiveRoleAssignment
import stp.rc as rc
import numpy as np
from typing import Dict, Generic, Iterator, List, Optional, Tuple, Type, TypeVar

class CaptureCost(role.CostFn):
    """
    A cost function for how to choose a robot that will pass
    """

    def __call__(self, robot: rc.Robot, prev_result: Optional["RoleResult"],
                 world_state: rc.WorldState) -> float:
        if robot.has_ball_sense:
            return 0
        else:
            robot_pos = robot.pose[0:2]
            ball_pos = world_state.ball.pos[0:2]
            dist_to_ball = np.linalg.norm(ball_pos - robot_pos)
            return dist_to_ball 


class Corner(play.IPlay):

    def __init__(self):
        self.target_point = [0.0, 10.0]
        self.goalie = goalie_tactic.GoalieTactic()
        # self.two_mark = nmark_tactic.NMarkTactic(2)
        self.move = move_tactic.Move([0.0, 8.5], face_point=[0.0, 12.0])
        self.clear = clear_tactic.Clear(np.array([0.0, 10.0]), chip=True, kick_speed=0.75)
        self.striker_tactic = striker_tactic.LineKickStrikerTactic(target_point=self.target_point)
        self.striker_tactic.capture_cost = CaptureCost()
        self.role_assigner = NaiveRoleAssignment()
        self.shoot = False

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
        if not self.shoot:
            role_requests[self.clear] = self.clear.get_requests(world_state, None)
            role_requests[self.move] = self.move.get_requests(world_state, None)
        else:
            role_requests[self.striker_tactic] = self.striker_tactic.get_requests(world_state, None)
            print("HEEEEERRRRRREEEEEEEEs")

        # role_requests[self.two_mark] = (self.two_mark.get_requests(world_state, None))
        role_requests[self.goalie] = self.goalie.get_requests(world_state, None)

        # Flatten requests and use role assigner on them
        flat_requests = play.flatten_requests(role_requests)
        flat_results = self.role_assigner.assign_roles(flat_requests, world_state, prev_results)
        role_results = play.unflatten_results(flat_results)

        # Get list of all skills with assigned roles from tactics
        # skills = self.two_mark.tick(role_results[self.two_mark])
        skills = []
        skill_dict = {}
        if not self.shoot:
            skills += self.clear.tick(role_results[self.clear], world_state)
            skills += self.move.tick(role_results[self.move])
            skill_dict.update(role_results[self.clear])
            skill_dict.update(role_results[self.move])
        else:
            print("BAAAAAADDDDDDDDDd")
            skills += self.striker_tactic.tick(role_results[self.striker_tactic], world_state)
            skill_dict.update(role_results[self.striker_tactic])
        
        skills += self.goalie.tick(role_results[self.goalie])
        
        # skill_dict.update(role_results[self.two_mark])
        # skill_dict.update(role_results[self.clear])
        skill_dict.update(role_results[self.goalie])

        if self.clear.is_done(world_state):
            print("HEREERERE")
            self.shoot = True

        return (skill_dict, skills)

    def is_done(self, world_state):
        return self.striker_tactic.is_done(world_state)
