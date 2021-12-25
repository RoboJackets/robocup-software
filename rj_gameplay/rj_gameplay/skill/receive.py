from abc import ABC, abstractmethod

import rj_gameplay.eval as eval
import argparse
import py_trees
import sys
import time
import numpy as np
from typing import Optional, Any, Dict, List, Type

import stp.skill as skill
import stp.role as role
from rj_gameplay.skill import settle, capture
from rj_msgs.msg import RobotIntent, SettleMotionCommand
import stp.rc as rc
from rj_msgs import msg

from rj_gameplay.action.move_action_client import MoveActionClient

"""
A skill version of receive so that actions don't have to be called in tactics
"""

class Receive(skill.ISkill):
    def __init__(self,
                 action_client_dict: Dict[Type[Any], List[Any]],
                 robot: rc.Robot = None):
        self.robot = robot

        self.move_action_clients = action_client_dict.get(MoveActionClient)
        self.__name__ = 'receive skill'
        self.settle = settle.Settle(robot)
        self.capture = capture.Capture(robot)

    def tick(self, robot: rc.Robot, world_state: rc.WorldState,
             intent: RobotIntent):
        if self.settle.is_done(world_state):
            return self.capture.tick(robot, world_state, intent)
        else:
            return self.settle.tick(robot, world_state, intent)


    def is_done(self, world_state:rc.WorldState) -> bool:

        return self.capture.is_done(world_state)


    def __str__(self):
        return f"Receive(robot={self.robot.id if self.robot is not None else '??'}, ticks={self.capture.ticks_done})"
