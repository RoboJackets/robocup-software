from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

import numpy as np
import stp
import stp.play as play
import stp.rc as rc
import stp.role as role
import stp.skill as skill
import stp.tactic as tactic
from rj_msgs.msg import RobotIntent
from stp.role.assignment.naive import NaiveRoleAssignment

from rj_gameplay.calculations import wall_calculations
from rj_gameplay.tactic import (
    goalie_tactic,
    move_tactic,
    nmark_tactic,
    striker_tactic,
    wall_tactic,
)


class State(Enum):
    INIT = auto()
    ACTIVE = auto()
    DONE = auto()


class PrepareKickoff(stp.play.Play):
    def __init__(self):
        super().__init__()
        self._state = State.INIT

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:

        if self._state == State.INIT:
            self.prioritized_tactics.append(goalie_tactic.GoalieTactic(world_state, 0))
            # TODO robot needs to go to pt in circle
            self.prioritized_tactics.append(wall_tactic.WallTactic(world_state, 4))
            self.assign_roles(world_state)
            self._state = State.ACTIVE
            return self.get_robot_intents(world_state)
        if self._state == State.ACTIVE:
            done = True
            for tactic in self.prioritized_tactics:
                if not tactic.is_done(world_state):
                    done = False
            if done:
                self._state = State.DONE
            return self.get_robot_intents(world_state)
        if self._state == State.DONE:
            return None


class Kickoff(stp.play.Play):
    def __init__(self):
        super().__init__()
        self._state = State.INIT

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:

        if self._state == State.INIT:
            self.prioritized_tactics.append(goalie_tactic.GoalieTactic(world_state, 0))
            self.prioritized_tactics.append(striker_tactic.StrikerTactic(world_state))
            self.prioritized_tactics.append(wall_tactic.WallTactic(world_state, 4))
            self.assign_roles(world_state)
            self._state = State.ACTIVE
            return self.get_robot_intents(world_state)
        if self._state == State.ACTIVE:
            if self.prioritized_tactics[-2].is_done(world_state):
                self._state = State.DONE
            return self.get_robot_intents(world_state)
        if self._state == State.DONE:
            return None


class DefendKickoff(stp.play.Play):
    def __init__(self):
        super().__init__()
        self._state = State.INIT

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:

        if self._state == State.INIT:
            self.prioritized_tactics.append(goalie_tactic.GoalieTactic(world_state, 0))
            self.prioritized_tactics.append(wall_tactic.WallTactic(world_state, 5))
            self.assign_roles(world_state)
            self._state = State.ACTIVE
            return self.get_robot_intents(world_state)
        elif self._state == State.ACTIVE:
            # return robot intents from assigned tactics back to gameplay node
            return self.get_robot_intents(world_state)
