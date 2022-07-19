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
    """This is a class that represents the status of the PenaltyOffense play."""
    INIT = auto()
    ACTIVE = auto()


# TODO: add a shared state between these two classes somehow
class PenaltyOffense(stp.play.Play):
    """This is a play that represents the lineup procedure in the event that our team has been awarded a penalty kick. 
       Per rule 3 in Section 5.3.5 of the RoboCup SSL Playbook (https://robocup-ssl.github.io/ssl-rules/sslrules.html#_penalty_kick),
       "Throughout the penalty kick procedure, all other robots have to be 1m behind the ball such that they do not interfere the penalty kick procedure".
       As such, this play lines up 5 offensive robots such that they are at least 1m behind the ball and assigns a striker to take the penalty.

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
    """This class, when called, prepares the PenaltyOffense play by lining up 5 offensive robots such that they are at least 1m behind the ball 
    and assigns a striker to take the penalty.

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
            self.prioritized_tactics.append(prep_move.PrepMove(world_state))
            num_liners = len(world_state.our_visible_robots) - 1
            # 7.5 meters to opponent's goal (6 meters to penalty line, 1 meter behind line, then spare distance to be safe)
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
