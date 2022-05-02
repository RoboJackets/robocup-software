from enum import Enum, auto
from typing import List

import numpy as np
import stp
import stp.play as play
import stp.rc as rc
import stp.tactic as tactic
from rj_msgs.msg import RobotIntent

from rj_gameplay.tactic import line_tactic, prep_move


class State(Enum):
    INIT = auto()
    PREP = auto()
    READY = auto()
    # DONE = auto()


class PenaltyOffense(stp.play.Play):
    def __init__(self):
        super().__init__()

        self._state = State.INIT

    def tick(
        self,
        world_state: rc.WorldState,
    ) -> List[RobotIntent]:

        if self._state == State.INIT:
            self.prioritized_tactics = [
                prep_move.PrepMove(world_state),
                line_tactic.LineTactic(
                    world_state, 5, np.array([2.0, 2.0]), np.array([-2.0, 2.0])
                ),
            ]
            self.assign_roles(world_state)
            self._state = State.PREP
            return self.get_robot_intents(world_state)

        elif self._state == State.PREP:
            for t in self.prioritized_tactics:
                t.tick(world_state)
            self.assign_roles(world_state)
            return self.get_robot_intents(world_state)

        # elif self._state == State.READY:  # TODO: add when it's ready
        #     self.prioritized_tactics = [
        #         goalie_tactic.GoalieTactic(world_state, 0),
        #         striker_tactic.StrikerTactic(world_state),
        #     ]
        #     shoot = self.prioritized_tactics[1]
        #     if shoot.is_done(world_state):
        #         self._state = State.DONE
        #     return self.get_robot_intents(world_state)
