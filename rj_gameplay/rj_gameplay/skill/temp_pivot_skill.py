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
from rj_gameplay.action import pivot
from stp.skill.action_behavior import ActionBehavior
import stp.rc as rc
from rj_msgs import msg

class IPivot(skill.ISkill, ABC):
    ...


"""
A skill version of pivot so that actions don't have to be called in tactics
"""
class Pivot(IPivot):
    
    def __init__(self,
            robot : rc.Robot = None,
            pivot_point: np.ndarray = np.array([0.0,0.0]),
            target_point : np.ndarray = np.array([0.0,0.0])):
        self.robot = robot
        self.pivot_point = pivot_point
        self.target_point = target_point
        if self.robot is not None:
            self.pivot = pivot.Pivot(self.robot.id, pivot_point, target_point)
        else:
            self.pivot = pivot.Pivot(self.robot, pivot_point, target_point)
        self.pivot_behavior = ActionBehavior('Pivot', self.pivot)
        self.root = self.pivot_behavior
        self.root.setup_with_descendants()
        self.__name__ = 'pivot skill'

    def tick(self, robot: rc.Robot, world_state): #returns dict of robot and actions
        self.robot = robot
        if np.linalg.norm(self.pivot.pivot_point - robot.pose[0:2]) > 0.3:
            # Without this the robot will moving a little to reach a new pivot point
            self.pivot.pivot_point = robot.pose[0:2]
        actions = self.root.tick_once(self.robot, world_state)
        return actions

    def is_done(self, world_state):
        return self.pivot.is_done(world_state)
