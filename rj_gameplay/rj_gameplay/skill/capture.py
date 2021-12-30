from abc import ABC, abstractmethod

import rj_gameplay.eval as eval
import argparse
import py_trees
import sys
import time
from rj_msgs.msg import RobotIntent

import stp.skill as skill
import stp.role as role
import stp.action as action
import stp.rc as rc
from typing import Optional

"""
A skill version of capture so that actions don't have to be called in tactics
"""


class Capture(skill.ISkill):  # add ABC if something fails
    def __init__(self, robot: Optional[rc.Robot] = None):
        self.robot = robot
        self.ticks_done = 0

        self.__name__ = "capture skill"

    def tick(
        self, robot: rc.Robot, world_state: rc.WorldState, intent: RobotIntent
    ):
        self.robot = robot
        collect_command = CollectMotionCommand()
        intent.motion_command.collect_command = [collect_command]
        intent.dribbler_speed = 1.0
        intent.is_active = True
        return {self.robot.id: intent}

    def is_done(self, world_state) -> bool:
        if (
            self.robot is not None
            and world_state.our_robots[self.robot.id].has_ball_sense
        ):
            self.ticks_done += 1
        else:
            self.ticks_done -= 5
        self.ticks_done = np.clip(self.ticks_done, a_min=0, a_max=200)
        return self.ticks_done > 50

    def __str__(self):
        return f"Capture(robot={self.robot.id if self.robot is not None else '??'})"
