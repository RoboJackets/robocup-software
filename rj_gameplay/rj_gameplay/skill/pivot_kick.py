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
from rj_gameplay.MAX_KICK_SPEED import *



class PivotKick(skill.ISkill): # add ABC if fails
    """
    A pivot kick skill
    capture -> pivot -> kick
    """

    # TODO: Have something which automatically determines kick speed based on target point distance
    def __init__(self,
                 robot: rc.Robot=None,
                 pivot_point: np.ndarray=None,
                 target_point: np.ndarray=None,
                 dribble_speed: float = 1,
                 chip: bool=False,
                 kick_speed: float=MAX_KICK_SPEED,
                 threshold: float = 0.02,
                 priority: int = 1) -> None:

        self.__name__ = 'pivot kick'
        self.robot = robot
        self.pivot_point = pivot_point
        self.target_point = target_point
        self.dribble_speed = dribble_speed
        self.chip = chip
        self.kick_speed = kick_speed
        self.threshold = threshold

        self.kick = kick.Kick(robot, chip, kick_speed, threshold)
        self.pivot = pivot.Pivot(robot, pivot_point, target_point, dribble_speed, threshold, priority)
        self.capture = capture.Capture(robot)


    def tick(self, robot: rc.Robot, world_state: rc.WorldState,
             intent: RobotIntent):
        if self.capture.is_done(world_state):
            # self.robot = self.pivot.tick.robot
            # self.pivot_point = self.pivot.tick.pivot_point
            return self.pivot.tick(robot, world_state, intent)
        elif self.pivot.is_done(world_state):
            return self.kick.tick(robot, world_state, intent)
        else:
            # self.robot = self.capture.tick.robot
            return self.capture.tick(robot, world_state, intent)

    def is_done(self, world_state: rc.WorldState) -> bool:
        return self.kick.is_done

    def __str__(self):
        return f"Capture(robot={self.capture.robot.id if self.capture.robot is not None else '??'}), Pivot(robot={self.pivot.robot.id if self.pivot.robot is not None else '??'}, target={self.pivot.target_point}), Kick(robot={self.kick.robot.id if self.kick.robot is not None else '??'})"
