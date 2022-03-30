from enum import Enum, auto
from typing import List

import stp.play
import stp.rc
import stp.role
import stp.role.cost
import stp.skill
import stp.tactic
from rj_msgs.msg import RobotIntent

from rj_gameplay.tactic import line_tactic


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
