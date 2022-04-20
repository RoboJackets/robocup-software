import argparse
import sys
import time
from enum import Enum, auto

import numpy as np
import stp.rc as rc
import stp.skill as skill
from rj_msgs.msg import RobotIntent
from stp.utils.constants import RobotConstants

from rj_gameplay.skill import capture, kick, pivot


class State(Enum):
    CAPTURE = auto()
    PIVOT = auto()
    KICK = auto()
    DONE = auto()


class PivotKick(skill.Skill):  # add ABC if fails
    """
    A pivot kick skill
    capture -> pivot -> kick
    """

    # TODO: Have something which automatically determines kick speed based on target point distance
    def __init__(
        self,
        robot: rc.Robot = None,
        pivot_point: np.ndarray = None,
        target_point: np.ndarray = None,
        dribble_speed: float = 1.0,
        chip: bool = False,
        kick_speed: float = RobotConstants.MAX_KICK_SPEED,
        threshold: float = 0.02,
        priority: int = 1,
    ) -> None:

        self.__name__ = "pivot kick"
        self.robot = robot
        # TODO: make skill take either np array or tuple (and auto-cast to whichever is best)
        self.pivot_point = pivot_point
        self.target_point = target_point
        self.dribble_speed = dribble_speed
        self.chip = chip
        self.kick_speed = kick_speed
        self.threshold = threshold

        self.kick = kick.Kick(robot, chip, kick_speed, threshold)
        self.pivot = pivot.Pivot(
            robot,
            pivot_point,
            target_point,
            dribble_speed,
            threshold,
            priority,
        )
        self.capture = capture.Capture(robot)
        self._state = State.CAPTURE

    def tick(self, world_state: rc.WorldState) -> RobotIntent:
        super().tick(world_state)

        intent = None
        if self._state == State.CAPTURE:
            intent = self.capture.tick(world_state)
            if self.capture.is_done(world_state):
                self._state = State.PIVOT
        elif self._state == State.PIVOT:
            intent = self.pivot.tick(world_state)
            if self.pivot.is_done(world_state):
                self._state = State.KICK
        elif self._state == State.KICK:
            intent = self.kick.tick(world_state)
            if self.kick.is_done(world_state):
                self._state = State.DONE

        return intent

    def is_done(self, world_state: rc.WorldState) -> bool:
        return self._state == State.DONE

    def __str__(self):
        return f"Pivot(robot={self.robot.id if self.robot is not None else '??'}, target={self.target_point})"

    def __repr__(self) -> str:
        return self.__str__()
