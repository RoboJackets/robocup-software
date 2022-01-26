from abc import ABC, abstractmethod

import rj_gameplay.eval as eval
import argparse
import py_trees
import sys
import time
import numpy as np
from typing import Optional

import stp.skill as skill
import stp.role as role
from rj_gameplay.skill import settle, capture
from rj_msgs.msg import RobotIntent, SettleMotionCommand
import stp.rc as rc
from rj_msgs import msg


class Receive(skill.Skill):
    """Skill that ends when robot has possession of the ball.
    Consists of Settle then Capture.
    """

    def __init__(self, robot: rc.Robot = None):
        self.robot = robot

        self.__name__ = "receive skill"
        self.settle = settle.Settle(robot)
        self.capture = capture.Capture(robot)

    def tick(self, world_state: rc.WorldState) -> RobotIntent:
        # TODO: put an FSM here instead of this obfuscated if-else
        if self.settle.is_done(world_state):
            return self.capture.tick(world_state)
        else:
            return self.settle.tick(world_state)

    def is_done(self, world_state: rc.WorldState) -> bool:

        return self.capture.is_done(world_state)

    def __str__(self):
        return f"Receive(robot={self.robot.id if self.robot is not None else '??'}, ticks={self.capture.ticks_done})"

    def __repr__(self) -> str:
        return self.__str__()
