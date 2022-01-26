from abc import ABC, abstractmethod

import rj_gameplay.eval as eval
import argparse
import py_trees
import sys
import time

import stp.skill as skill
import stp.role as role
from rj_gameplay.skill import kick, pivot, capture
from rj_msgs.msg import RobotIntent
import stp.rc as rc
import numpy as np
from rj_gameplay.MAX_KICK_SPEED import MAX_KICK_SPEED


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
        kick_speed: float = MAX_KICK_SPEED,
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

        self._state = "capture"

    def tick(self, world_state: rc.WorldState) -> RobotIntent:
        print("pivot kick state:", self._state)

        intent = None
        if self._state == "capture":
            intent = self.capture.tick(world_state)
            if self.capture.is_done(world_state):
                self._state = "pivot"
        elif self._state == "pivot":
            intent = self.pivot.tick(world_state)
            if self.pivot.is_done(world_state):
                self._state = "kick"
        elif self._state == "kick":
            intent = self.kick.tick(world_state)
            if self.kick.is_done(world_state):
                self._state = "done"

        return intent

    def is_done(self, world_state: rc.WorldState) -> bool:
        return self._state == "done"

    def __str__(self):
        return f"Pivot(robot={self.robot.id if self.robot is not None else '??'}, target={self.target_point})"

    def __repr__(self) -> str:
        return self.__str__()
