from enum import Enum, auto
from typing import List

import numpy as np
import stp
import stp.rc
import stp.role
import stp.role.cost
from rj_msgs.msg import RobotIntent

from rj_gameplay.tactic import line_tactic, prep_move, striker_tactic


class State(Enum):
    INIT = auto()
    ACTIVE = auto()


# TODO: add a shared state between these two classes somehow
class PenaltyOffense(stp.play.Play):
    """striker + line up rest of robots"""

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
            start_pt = np.array([-2.4, 1.5])
            end_pt = np.array([2.4, 1.5])
            self.prioritized_tactics.append(
                line_tactic.LineTactic(world_state, num_liners, start_pt, end_pt)
            )
            self.assign_roles(world_state)
            self._state = State.ACTIVE
            return self.get_robot_intents(world_state)
        elif self._state == State.ACTIVE:
            # return robot intents from assigned tactics back to gameplay node
            return self.get_robot_intents(world_state)


class PrepPenaltyOff(stp.play.Play):
    """prepare striker + line up all robots"""

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
            # 7.5 metres to opponent's goal (6 metres to penalty line, 1 metre behind line, then spare distance to be safe)
            start_pt = np.array([-2.4, 1.5])
            end_pt = np.array([2.4, 1.5])
            self.prioritized_tactics.append(
                line_tactic.LineTactic(world_state, num_liners, start_pt, end_pt)
            )
            self.assign_roles(world_state)
            self._state = State.ACTIVE
            return self.get_robot_intents(world_state)
        elif self._state == State.ACTIVE:
            # return robot intents from assigned tactics back to gameplay node
            return self.get_robot_intents(world_state)
