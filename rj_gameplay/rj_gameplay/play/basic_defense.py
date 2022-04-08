import stp

from rj_gameplay.tactic import wall_tactic, goalie_tactic, nmark_tactic
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


class BasicDefense(stp.play.Play):
    """Play that consists of:
    - 1 Goalie
    - 5 Wallers
    TODO: add 2 aggressive markers, go down to 3 Wallers
    """

    def __init__(self):
        super().__init__()

        self._state = State.INIT

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:

        #print(self._state)
        if self._state == State.INIT:
            self.prioritized_tactics.append(goalie_tactic.GoalieTactic(world_state, 4))
            self.prioritized_tactics.append(wall_tactic.WallTactic(world_state, 2))
            #self.prioritized_tactics.append(nmark_tactic.NMarkTactic(world_state, 2))
            self.assign_roles(world_state)
            self._state = State.ACTIVE
            return self.get_robot_intents(world_state)
        elif self._state == State.ACTIVE:
            # return robot intents from assigned tactics back to gameplay node
            return self.get_robot_intents(world_state)
