import stp.play
import stp.tactic

from rj_gameplay.tactic import line_tactic
import stp.skill
import stp.role
import stp.role.cost
from stp.role.assignment.naive import NaiveRoleAssignment
import stp.rc
from typing import (
    Dict,
    List,
    Tuple,
    Optional,
    Type,
)
import numpy as np
from rj_msgs.msg import RobotIntent

from enum import Enum, auto


class State(Enum):
    INIT = auto()
    LINE_UP = auto()
    DONE = auto()


class LineUp(stp.play.Play):
    """Lines up all six robots on the side of the field."""

    def __init__(self):
        super().__init__()

        self._state = State.INIT

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:

        if self._state == State.INIT:
            self.prioritized_tactics.append(line_tactic.LineTactic(world_state, 6))
            self.assign_roles(world_state)
            self._state = State.LINE_UP
            return self.get_robot_intents(world_state)
        elif self._state == State.LINE_UP:
            if self.prioritized_tactics[0].is_done(world_state):
                self._state = State.DONE
            return self.get_robot_intents(world_state)
        elif self._state == State.DONE:
            # TODO: does this state need to exist?
            return None
