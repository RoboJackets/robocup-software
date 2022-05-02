from abc import ABC, abstractmethod

import rj_gameplay.eval as eval
import argparse
import py_trees
import sys
import time
import numpy as np
from rj_msgs.msg import RobotIntent, SettleMotionCommand

import stp.skill as skill
import stp.role as role
import stp.rc as rc
from typing import Optional

SETTLE_BALL_SPEED_THRESHOLD = 0.75


class Settle(skill.Skill):
    """First half of a Receive Skill. Slows the ball down to allow Capture planner to work."""

    def __init__(self, robot: rc.Robot = None):
        self.robot = robot

        self.__name__ = "settle skill"

    def tick(self, world_state: rc.WorldState) -> RobotIntent:
        super().tick(world_state)
        intent = RobotIntent()

        settle_command = SettleMotionCommand()
        intent.motion_command.settle_command = [settle_command]
        intent.dribbler_speed = 1.0
        intent.is_active = True

        return intent

    def is_done(self, world_state) -> bool:
        if self.robot is None:
            return False
        if (
            world_state.our_robots[self.robot.id].has_ball_sense
            or np.linalg.norm(world_state.ball.vel) < SETTLE_BALL_SPEED_THRESHOLD
        ):
            return True
        return False

    def __str__(self):
        return f"Capture(robot={self.robot.id if self.robot is not None else '??'})"

    def __repr__(self) -> str:
        return self.__str__()
