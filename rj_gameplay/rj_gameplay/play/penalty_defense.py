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
    """This class represents the state of the PenaltyDefense play."""
    INIT = auto()
    ACTIVE = auto()

class PenaltyDefense(stp.play.Play):
    """This is a play that represents the lineup procedure in the event that an opposing team has been awarded a penalty kick.
       Per rule 3 in Section 5.3.5 of the RoboCup SSL Playbook (https://robocup-ssl.github.io/ssl-rules/sslrules.html#_penalty_kick),
       this play assigns one goalkeeper and lines up the other bots at least 1m behind the ball.

    :param state: An enum value that represents the current state of the play.
    :type state: class:`State`
    :param world_state: Contains the states of all aspects of the match.
    :type world_state: class:`stp.rc.WorldState`
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
            num_liners = len(world_state.our_visible_robots) - 1
            # 7.5 meters to our goal (designed so that 5 robots can be at least 1 meter behind the penalty line before the penalty is taken)
            start_pt = np.array([-2.1, 7.25])
            end_pt = np.array([2.1, 7.25])
            self.prioritized_tactics.append(
                line_tactic.LineTactic(world_state, num_liners, start_pt, end_pt)
            )
            self.assign_roles(world_state)
            self._state = State.ACTIVE
            return self.get_robot_intents(world_state)
        elif self._state == State.ACTIVE:
            # return robot intents from assigned tactics back to gameplay node
            return self.get_robot_intents(world_state)
