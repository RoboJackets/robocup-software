from enum import Enum, auto
from typing import List

import numpy as np
import stp
import stp.rc
import stp.role
import stp.role.cost
from rj_msgs.msg import RobotIntent

from rj_gameplay.tactic import goalie_tactic, line_tactic


class State(Enum):
    INIT = auto()
    ACTIVE = auto()


class PenaltyDefense(stp.play.Play):
    def __init__(self):
        super().__init__()
        self._state = State.INIT

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:
        if self._state == State.INIT:
            self.prioritized_tactics.append(goalie_tactic.GoalieTactic(world_state, 0))
            num_liners = len(world_state.our_visible_robots) - 1
            start_pt = np.array([-3.0, 0.5])
            end_pt = np.array([-3.0, 5.5])
            self.prioritized_tactics.append(
                line_tactic.LineTactic(world_state, num_liners, start_pt, end_pt)
            )
            self.assign_roles(world_state)
            self._state = State.ACTIVE
            return self.get_robot_intents(world_state)
        elif self._state == State.ACTIVE:
            # return robot intents from assigned tactics back to gameplay node
            return self.get_robot_intents(world_state)
