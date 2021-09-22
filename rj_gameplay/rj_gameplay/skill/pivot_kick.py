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




class IPivotKick(skill.ISkill, ABC):
    ...


class PivotKick(IPivotKick):
    """
    A pivot kick skill
    capture -> pivot -> kick
    """

    def __init__(self, robot: rc.Robot) -> None:
        # TODO: Have something which automatically determines kick speed based on target point distance
        self.__name__ = 'pivot kick'
        self.robot = robot


    def tick(self, 
             robot: rc.Robot, 
             world_state: rc.WorldState, 
             intent: RobotIntent):
        if kick.is_done(world_state):
            return capture.tick(robot, world_state, intent)
        elif pivot.is_done(world_state:
            return kick.tick(robot, world_state, intent)
        else: 
            return pivot.tick(robot, world_state, intent)

    def is_done(self, world_state: rc.WorldState) -> bool:
        return self.capture.is_done

    def __str__(self):
        return f"Pivot(robot={self.robot.id if self.robot is not None else '??'}, target={self.target_point})"
