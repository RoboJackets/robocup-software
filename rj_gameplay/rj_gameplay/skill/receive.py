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
import stp.action as action
from rj_gameplay.action import receive, capture
from stp.skill.action_behavior import ActionBehavior
from stp.skill.rj_sequence import RjSequence as Sequence
import stp.rc as rc
from rj_msgs import msg

class IReceive(skill.ISkill, ABC):
    ...


"""
A skill version of receive so that actions don't have to be called in tactics
"""
class Receive(IReceive):
    
    def __init__(self,
            robot:rc.Robot = None):

        self.robot = robot
        if self.robot is not None:
            self.receive = receive.Receive(self.robot.id)
            self.capture = capture.Capture(self.robot.id)
        else:
            self.receive = receive.Receive()
            self.capture = capture.Capture()
        self.receive_behavior = ActionBehavior('Receive', self.receive)
        self.capture_behavior = ActionBehavior('Capture', self.capture)
        self.root = Sequence('Sequence')
        self.root.add_children([self.receive_behavior, self.capture_behavior])
        self.root.setup_with_descendants()
        self.__name__ = 'receive skill'

    def tick(self, robot:rc.Robot, world_state:rc.WorldState): #returns dict of robot and actions
        self.robot = robot
        actions = self.root.tick_once(self.robot, world_state)
        self.capture.robot_id = self.robot.id
        self.receive.robot_id = self.robot.id
        return actions

    def is_done(self, world_state:rc.WorldState):
        return self.capture.is_done(world_state)
