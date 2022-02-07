from typing import List

import stp

from rj_gameplay.tactic import pass_tactic

from rj_msgs.msg import RobotIntent

from enum import Enum, auto


class State(Enum):
    INIT = auto()
    ACTIVE = auto()
    ASSIGN_ROLES = auto()


class Basic122(stp.play.Play):
    """Basic play to score goals. Set up two flank and one center handler. Pass ball as needed.
    See tick() for more details.
    """

    def __init__(self):
        super().__init__()

        self._state = State.INIT

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:
        """
        init: 1 ball handler, 2 seekers, 2 markers, 1 goalie
        when ready: ball handler shoots or passes with 1/3rd probability
        if ball handler passes: passer shoots immediately

        """
