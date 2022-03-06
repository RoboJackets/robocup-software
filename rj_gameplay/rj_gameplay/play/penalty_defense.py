from rj_msgs.msg import RobotIntent

import stp
import stp.tactic
import stp.role
import stp.rc
import stp.role.cost
import stp.skill

from stp.role.assignment.naive import NaiveRoleAssignment

from rj_gameplay.tactic import line_tactic, goalie_tactic
from rj_gameplay.calculations import wall_calculations

from typing import Dict, List, Tuple, Type
from enum import Enum, auto


class State(Enum):
    INIT = auto()
    ACTIVE = auto()
    DONE = auto()


class PenaltyDefense(stp.play.Play):
    """
    Play consistas of:
            -5 wallers
            -1 goalie
    The goalie will defend the opponents penalty kick.
    """

    def __init__(self):
        super().__init__()
        self._state = State.INIT

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:
        if self._state == State.INIT:
            self.prioritized_tactics.append(line_tactic.LineTactic(world_state, 6))
            ##self.prioritized_tactics.append(goalie_tactic.GoalieTactic(world_state, 0))
            self.assign_roles(world_state)
            self._state = State.ACTIVE
            return self.get_robot_intents(world_state)
        elif self._state == State.ACTIVE:
            if self.prioritized_tactics[-1].is_done(world_state):
                self._state = State.DONE
            return self.get_robot_intents(world_state)
        elif self._state == State.Done:
            # TODO: does this state need to exist?
            return None


class PreparePenaltyDefense(stp.play.Play):
    def __init(self):
        super().__init__()
        self._state = State.Init

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:
        if self._state == State.INIT:
            self.prioritized_tactics.append(line_tactic.LineTactic(world_state, 6))
            self.assign_roles(world_state)
            self._state = State.ACTIVE
            return self.get_robot_intents(world_state)
        elif self._state == State.ACTIVE:
            if self.prioritized_tactics[-1].is_done(world_state):
                self._state = State.DONE
            return self.get_robot_intents(world_state)
        elif self._state == State.Done:
            # TODO: does this state need to exist?
            return None
