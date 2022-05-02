from enum import Enum, auto
from typing import List

import stp
import stp.rc
import stp.role
import stp.role.cost
from rj_msgs.msg import RobotIntent

from rj_gameplay.tactic import goalie_tactic, nmark_tactic, wall_tactic


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

        if self._state == State.INIT:
            self.prioritized_tactics.append(goalie_tactic.GoalieTactic(world_state, 0))
            num_wallers = min(3, len(world_state.our_visible_robots) - 1)
            self.prioritized_tactics.append(
                wall_tactic.WallTactic(world_state, num_wallers)
            )
            num_markers = len(world_state.our_visible_robots) - (1 + num_wallers)
            self.prioritized_tactics.append(
                nmark_tactic.NMarkTactic(world_state, num_markers)
            )
            self.assign_roles(world_state)
            self._state = State.ACTIVE
            return self.get_robot_intents(world_state)
        elif self._state == State.ACTIVE:
            # return robot intents from assigned tactics back to gameplay node
            return self.get_robot_intents(world_state)
