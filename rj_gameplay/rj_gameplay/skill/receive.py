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
from rj_gameplay.action import receive
from stp.skill.action_behavior import ActionBehavior
import stp.rc as rc
from rj_msgs import msg

class IReceive(skill.ISkill, ABC):
    ...


"""
A skill version of receive so that actions don't have to be called in tactics
"""
class Receive(IReceive):
    
    def __init__(self,
            robot:rc.Robot = None,
            one_touch_target:np.ndarray = None):

        self.robot = robot
        self.one_touch_target = one_touch_target
        if self.robot is not None:
            self.receive = receive.Receive(self.robot.id, self.one_touch_target)
        else:
            self.receive = receive.Receive(self.robot, self.one_touch_target)
        self.receive_behavior = ActionBehavior('Receive', self.receive)
        self.root = self.receive_behavior
        self.root.setup_with_descendants()
        self.__name__ = 'move skill'

    def tick(self, robot:rc.Robot, world_state:rc.WorldState): #returns dict of robot and actions
        self.robot = robot
        actions = self.root.tick_once(self.robot, world_state)
        return actions

    def is_done(self, world_state:rc.WorldState):
        return self.receive.is_done(world_state)
