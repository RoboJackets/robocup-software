from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

import numpy as np
import stp
import stp.play as play
import stp.rc as rc
import stp.role as role
import stp.role.cost as cost
import stp.skill as skill
import stp.tactic as tactic
from rj_msgs.msg import RobotIntent
from stp.role.assignment.naive import NaiveRoleAssignment

from rj_gameplay.tactic import (
    dumb_tactic,
    goalie_tactic,
    move_tactic,
    nmark_tactic,
    pass_tactic,
    striker_tactic,
    wall_tactic,
)


class State(Enum):
    INIT = auto()
    ACTIVE = auto()
    ASSIGN_ROLES = auto()
    DONE = auto()


class PrepareKickoff(stp.play.Play):
    def __init__(self):
        super().__init__()
        self.pts = []
        self._state = State.INIT

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:
        if self._state == State.INIT:
            if len(self.pts) == 0:
                self.create_pts(world_state)
            self.prioritized_tactics.append(goalie_tactic.GoalieTactic(world_state, 0))
            self.prioritized_tactics.append(
                dumb_tactic.DumbTactic(world_state, self.pts)
            )
            self.prioritized_tactics.append(wall_tactic.WallTactic(world_state, 3))
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
            return self.get_robot_intents(world_state)

    def create_pts(self, world_state):
        self.pts = []
        self.pts.append(
            (
                0.0,
                (world_state.field.length_m - world_state.field.center_radius_m) / 2,
            )
        )
        dx = 0.5
        dy = -0.5
        self.pts.append((self.pts[0][0] + dx, self.pts[0][1] + dy))


class Kickoff(stp.play.Play):
    def __init__(self):
        super().__init__()
        self.pts = []
        self._state = State.INIT

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:
        if self._state == State.INIT:
            if len(self.pts) == 0:
                self.create_pts(world_state)
            self.prioritized_tactics.append(goalie_tactic.GoalieTactic(world_state, 0))
            self.prioritized_tactics.append(wall_tactic.WallTactic(world_state, 3))
            self.prioritized_tactics.append(
                pass_tactic.PassTactic(
                    world_state,
                    cost.PickClosestToPoint(self.pts[0]),
                    cost.PickClosestToPoint(self.pts[1]),
                )
            )
            # self.prioritized_tactics.append(dumb_tactic.DumbTactic(world_state, [self.pts[1]]))
            self.assign_roles(world_state)
            self._state = State.ACTIVE
            return self.get_robot_intents(world_state)
        elif self._state == State.ACTIVE:
            for tactic in self.prioritized_tactics:
                if tactic.needs_assign:
                    self._state = State.ASSIGN_ROLES
            return self.get_robot_intents(world_state)
        elif self._state == State.ASSIGN_ROLES:
            self.assign_roles(world_state)
            self._state = State.ACTIVE
            return self.get_robot_intents(world_state)
        elif self._state == State.DONE:
            return self.get_robot_intents(world_state)

    def create_pts(self, world_state):
        # pts[0] is near the midpoint of the field
        # pts[1] is diagonal to pts[0]
        self.pts = []
        self.pts.append(
            (0.0, (world_state.field.length_m - world_state.field.center_radius_m) / 2)
        )
        dx = 0.5
        dy = -0.5
        self.pts.append((self.pts[0][0] + dx, self.pts[0][1] + dy))


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
            return self.get_robot_intents(world_state)
