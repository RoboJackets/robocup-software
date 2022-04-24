import stp

from rj_gameplay.tactic import wall_tactic, goalie_tactic, nmark_tactic, line_tactic, striker_tactic, prep_move
import stp.role
from stp.role.assignment.naive import NaiveRoleAssignment
import stp.rc
from typing import Dict, List, Tuple, Type
from rj_gameplay.calculations import wall_calculations

import stp.role.cost
from rj_msgs.msg import RobotIntent

from enum import Enum, auto


class State(Enum):
    INIT = auto()
    ACTIVE = auto()


class PenaltyOffense(stp.play.Play):
    """ striker + line up rest of robots """
    def __init__(self):
        super().__init__()

        self._state = State.INIT

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:

        if self._state == State.INIT:
            self.prioritized_tactics.append(striker_tactic.StrikerTactic(world_state))
            num_liners = len(world_state.our_visible_robots) - 1
            self.prioritized_tactics.append(line_tactic.LineTactic(world_state, num_liners))
            self.assign_roles(world_state)
            self._state = State.ACTIVE
            return self.get_robot_intents(world_state)
        elif self._state == State.ACTIVE:
            # return robot intents from assigned tactics back to gameplay node
            return self.get_robot_intents(world_state)

class PrepPenaltyOff(stp.play.Play):
    """ prepare striker + line up all robots """
    def __init__(self):
        super().__init__()

        self._state = State.INIT

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:

        if self._state == State.INIT:
            self.prioritized_tactics.append(prep_move.PrepMove(world_state))
            num_liners = len(world_state.our_visible_robots) - 1
            self.prioritized_tactics.append(line_tactic.LineTactic(world_state, num_liners))
            self.assign_roles(world_state)
            self._state = State.ACTIVE
            return self.get_robot_intents(world_state)
        elif self._state == State.ACTIVE:
            # return robot intents from assigned tactics back to gameplay node
            return self.get_robot_intents(world_state)
