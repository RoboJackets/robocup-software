from enum import Enum, auto
from typing import List

import numpy as np
import stp
import stp.rc
import stp.role
import stp.role.cost
from rj_msgs.msg import RobotIntent

from rj_gameplay.skill import move, pivot_kick


class State(Enum):
    NEAR = auto()
    FAR = auto()


class OneRobot(stp.play.Play):
    """
    Make robot 0 run full field sprints.

    Directly overrides the STP architecture to send RI to gameplay.
    """

    def __init__(self):
        super().__init__()

        self._state = State.NEAR
        self.move_skill = None

        self.robot_id = 1
        self.robot = None

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:
        self.robot = world_state.our_robots[self.robot_id]
        intents = [None for _ in range(16)]

        if self._state == State.NEAR:
            self.target_point = np.array([1.0, 3.5])
            self.face_point = np.array([0.0, 6.0])

            if self.move_skill is not None and self.move_skill.is_done(world_state):
                self._state = State.FAR
                self.move_skill = None

        elif self._state == State.FAR:
            self.target_point = np.array([-1.0, 3.5])
            self.face_point = np.array([0.0, 6.0])

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

        # TODO: only making this intent change when the move skill is reinitialized produces much smoother behavior, but makes our planning unable to respond to moving obstacles
        # I think skills should be in C++, so we can actually use the replanner (currently all planning is from scratch because we send fresh move intents every tick)
        intents[self.robot_id] = self.move_skill.tick(world_state)

        return intents


class AllBotsState(Enum):
    INIT_NEAR = auto()
    NEAR = auto()
    INIT_FAR = auto()
    FAR = auto()


class AllBots(stp.play.Play):
    """
    Make all robots move from one end to the other in straight line paths.

    Directly overrides the STP architecture to send RI to gameplay.
    """

    def __init__(self):
        super().__init__()

        self._state = AllBotsState.INIT_NEAR

        self._move_skills = None

        self._offset = 2.0

    def all_moves_done(self, world_state):
        return self._move_skills and all(
            [move_skill.is_done(world_state) for move_skill in self._move_skills]
        )

    def fill_move_skills(self, world_state, st_pt, end_pt):
        self._move_skills = []
        for i, pt in enumerate(
            np.linspace(st_pt, end_pt, len(world_state.our_visible_robots))
        ):
            self._move_skills.append(
                move.Move(
                    robot=world_state.our_robots[i],
                    target_point=pt,
                    face_point=pt,
                )
            )

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:
        if self._state == AllBotsState.INIT_NEAR:
            # evenly space pts along our goal side
            st_pt = np.array([world_state.field.width_m / -2, self._offset])
            end_pt = np.array([world_state.field.width_m / 2, self._offset])
            self.fill_move_skills(world_state, st_pt, end_pt)

            self._state = AllBotsState.NEAR

        elif self._state == AllBotsState.NEAR:
            if self.all_moves_done(world_state):
                self._state = AllBotsState.INIT_FAR

        elif self._state == AllBotsState.INIT_FAR:
            # evenly space pts along their goal side
            st_pt = np.array(
                [
                    world_state.field.width_m / -2,
                    world_state.field.length_m - self._offset,
                ]
            )
            end_pt = np.array(
                [
                    world_state.field.width_m / 2,
                    world_state.field.length_m - self._offset,
                ]
            )
            self.fill_move_skills(world_state, st_pt, end_pt)

            self._state = AllBotsState.FAR

        elif self._state == AllBotsState.FAR:
            if self.all_moves_done(world_state):
                self._state = AllBotsState.INIT_NEAR

        intents = [None for _ in range(16)]

        if self._move_skills is not None:
            # tick move skills to fill RobotIntents
            for i, move_skill in enumerate(self._move_skills):
                intents[i] = self._move_skills[i].tick(world_state)

        return intents


class KickBall(stp.play.Play):
    """
    Make robot 0 capture, then kick the ball.

    Directly overrides the STP architecture to send RI to gameplay.
    """

    def __init__(self):
        super().__init__()
        self._state = "capturing"
        self.capture_skill = None
        self.pivot_kick_skill = None

        self.robot_id = 1
        self.robot = None

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:
        self.robot = world_state.our_robots[self.robot_id]
        intents = [None for _ in range(16)]
        my_robot_intent = None

        # None on INIT and state changes
        if self.pivot_kick_skill is None:
            self.pivot_kick_skill = pivot_kick.PivotKick(
                robot=self.robot,
                target_point=np.array([0.0, 6.0]),
                chip=True,
                kick_speed=5.0,
            )
        else:
            my_robot_intent = self.pivot_kick_skill.tick(world_state)

        intents[self.robot_id] = my_robot_intent

        return intents
