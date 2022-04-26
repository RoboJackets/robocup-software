from enum import Enum, auto
from typing import Dict, List, Tuple, Type

import numpy as np
import stp
import stp.rc
import stp.role
import stp.role.cost
from rj_msgs.msg import RobotIntent
from stp.role.assignment.naive import NaiveRoleAssignment

from rj_gameplay.calculations import wall_calculations
from rj_gameplay.skill import move
from rj_gameplay.tactic import goalie_tactic, nmark_tactic, wall_tactic


class State(Enum):
    NEAR = auto()
    FAR = auto()


class PlanningTestPlay(stp.play.Play):
    """
    Make robot 0 run full field sprints.
    """

    def __init__(self):
        super().__init__()

        self._state = State.NEAR
        self.move_skill = None

        self.robot_id = 0
        self.robot = None

        self.face_point = np.array([0.0, 0.0])

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:

        self.robot = world_state.our_robots[self.robot_id]

        if self._state == State.NEAR:
            self.target_point = np.array([2.0, 0.5])
            # self.face_point = self.target_point

            if self.move_skill is not None and self.move_skill.is_done(world_state):
                self._state = State.FAR
                self.move_skill = None

        elif self._state == State.FAR:
            self.target_point = np.array([2.0, 8.5])
            # self.face_point = self.target_point

            if self.move_skill is not None and self.move_skill.is_done(world_state):
                self._state = State.NEAR
                self.move_skill = None

        # None on INIT and state changes
        if self.move_skill is None:
            self.move_skill = move.Move(
                robot=self.robot,
                target_point=self.target_point,
                face_point=self.face_point,
            )

        intent = self.move_skill.tick(world_state)
        intents = [intent] + [None for _ in range(15)]
        return intents
