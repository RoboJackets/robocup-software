from enum import Enum, auto
from typing import Dict, List, Tuple, Type

import numpy as np
import stp
import stp.play as play
import stp.rc as rc
import stp.role as role
import stp.skill as skill
import stp.tactic as tactic
from rj_msgs.msg import RobotIntent

from rj_gameplay.tactic import goalie_tactic, line_tactic, prep_move, striker_tactic


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
                prep_move.PrepMove(world_state)
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
            self.prioritized_tactics[0].tick(world_state)
            move = self.prioritized_tactics[0]
            self.assign_roles(world_state)
            if move.is_done(world_state):
                self._state = State.READY
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
