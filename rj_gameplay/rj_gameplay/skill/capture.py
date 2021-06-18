from abc import ABC, abstractmethod

import rj_gameplay.eval as eval
import argparse
import py_trees
import sys
import time

import stp.skill as skill
import stp.role as role
import stp.action as action
from rj_gameplay.action import capture
from stp.skill.action_behavior import ActionBehavior
import stp.rc as rc
from typing import Optional

class ICapture(skill.ISkill, ABC):
    ...


"""
A skill version of capture so that actions don't have to be called in tactics
"""
class Capture(ICapture):
    
    def __init__(self, robot: Optional[rc.Robot]=None):
        self.robot = robot
        self.__name__ = 'Capture'
        self.capture = capture.Capture()
        self.capture_behavior = ActionBehavior('Capture', self.capture, self.robot)
        self.root = self.capture_behavior
        self.root.setup_with_descendants()

    def tick(self, robot: rc.Robot, world_state: rc.WorldState):
        self.robot = robot
        return self.root.tick_once(robot, world_state)

    def is_done(self, world_state) -> bool:
    	return self.capture.is_done(world_state)
