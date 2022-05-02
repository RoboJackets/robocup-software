from enum import Enum, auto
from typing import List

import stp
import stp.rc
import stp.role
import stp.role.cost
import stp.skill
import stp.tactic
from rj_msgs.msg import RobotIntent

from rj_gameplay.tactic import goalie_tactic, line_tactic


class State(Enum):
    INIT = auto()
    ACTIVE = auto()


class PenaltyDefense(stp.play.Play):
    """
    Play consists of:
                    -5 robots lined up on the side
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
            self.prioritized_tactics.append(goalie_tactic.GoalieTactic(world_state, 0))
            self.prioritized_tactics.append(line_tactic.LineTactic(world_state, 5))
            self.assign_roles(world_state)
            self._state = State.ACTIVE
            return self.get_robot_intents(world_state)
        elif self._state == State.ACTIVE:
            return self.get_robot_intents(world_state)
