import stp.play as play
import stp.tactic as tactic

from rj_gameplay.tactic import striker_tactic, goalie_tactic, line_tactic, basic_seek
import stp.skill as skill
import stp.role as role
import stp.rc as rc
import stp
from rj_msgs.msg import RobotIntent
from typing import (
    Dict,
    List,
    Tuple,
    Type,
)
import numpy as np
from enum import Enum, auto


class State(Enum):
    INIT = auto()
    PREP = auto()
    READY = auto()
    DONE = auto()


class PenaltyOffense(stp.play.Play):
    def __init__(self):
        super().__init__()

        self._state = State.INIT
        self._striker_pos = rc.WorldState.ball.pos[0:2] - [0, 0.2]

    def tick(
        self,
        world_state: rc.WorldState,
    ) -> List[RobotIntent]:
        if self._state == state.INIT:
            self.prioritized_tactics = [
                basic_seek.BasicSeek(self._striker_pos, world_state),
                # assume line tactic is working
                # line_tactic.LineTactic(world_state),
                # line_tactic.LineTactic(world_state),
                # line_tactic.LineTactic(world_state),
                # line_tactic.LineTactic(world_state),
                # line_tactic.LineTactic(world_state),
            ]
            self.assign_roles(world_state)
            self._state = State.PREP
            return self.get_robot_intents(world_state)

        elif self._state == State.PREP:
            move = self.prioritized_tactics[1]
            if move.is_done(world_state):
                self._state = State.READY
            return self.get_robot_intents(world_state)

        elif self._state == State.READY:  # TODO: add when it's ready
            self.prioritized_tactics = [
                goalie_tactic.GoalieTactic(world_state, 0),
                striker_tactic.StrikerTactic(world_state),
            ]
            shoot = self.prioritized_tactics[1]
            if shoot.is_done(world_state):
                self._state = State.DONE
            return self.get_robot_intents(world_state)


# class PenaltyOffense(play.IPlay):
#     """Move all robots to our half, but away from ball to prep for penalty kick"""

#     def __init__(self):
#         self.tactics = [
#             goalie_tactic.GoalieTactic(True),
#             striker_tactic.LineKickStrikerTactic(None),
#             move_tactic.Move((1.8, 0.0), priority=role.Priority.LOW),
#             move_tactic.Move((2.1, 0.0), priority=role.Priority.LOW),
#             move_tactic.Move((2.4, 0.0), priority=role.Priority.LOW),
#             move_tactic.Move((2.7, 0.0), priority=role.Priority.LOW),
#         ]
#         self.role_assigner = NaiveRoleAssignment()

#     def compute_props(self, prev_props):
#         pass

#     def tick(
#         self,
#         world_state: rc.WorldState,
#         prev_results: role.assignment.FlatRoleResults,
#         props,
#     ) -> Tuple[
#         Dict[Type[tactic.SkillEntry], List[role.RoleRequest]],
#         List[tactic.SkillEntry],
#     ]:
#         # Get role requests from all tactics and put them into a dictionary
#         role_requests: play.RoleRequests = {
#             tactic: tactic.get_requests(world_state, None) for tactic in self.tactics
#         }
#         # Flatten requests and use role assigner on them
#         flat_requests = play.flatten_requests(role_requests)
#         flat_results = self.role_assigner.assign_roles(
#             flat_requests, world_state, prev_results
#         )
#         role_results = play.unflatten_results(flat_results)
#         # Get list of all SkillEntries from all tactics
#         skills = []
#         tac_index = 0
#         for tactic in self.tactics:
#             if tac_index == 1:  # TODO BAD HACK
#                 skills += tactic.tick(world_state, role_results[tactic])
#             else:
#                 skills += tactic.tick(world_state, role_results[tactic])
#             tac_index += 1
#         # Get all role assignments
#         # SkillEntry to (list of?) RoleResult
#         skill_dict = {}
#         for tactic in self.tactics:
#             skill_dict.update(role_results[tactic])
#         return (skill_dict, skills)

#     def is_done(self, world_state):
#         return self.tactics[-1].is_done(world_state)
