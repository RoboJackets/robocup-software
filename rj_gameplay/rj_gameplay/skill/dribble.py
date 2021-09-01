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
from rj_gameplay.action import dribble
from stp.skill.action_behavior import ActionBehavior
import stp.rc as rc
from rj_msgs import msg


"""
A simple skill wrapper version of dribble so that actions don't have to be called in tactics
"""
class Dribble(skill.ISkill):

    def __init__(self,
            robot : rc.Robot = None,
            target_point : np.ndarray = np.array([0.0,0.0]),
            target_vel : np.ndarray = np.array([0.0,0.0]),
            face_angle : Optional[float] = None,
            face_point : Optional[np.ndarray] = None):
        self.robot = robot
        self.target_point = target_point
        if self.robot is not None:
            self.dribble = dribble.Dribble(self.robot.id, target_point, target_vel, face_angle, face_point)
        else:
            self.dribble = dribble.Dribble(self.robot, target_point, target_vel, face_angle, face_point)
        self.dribble_behavior = ActionBehavior('Dribble', self.dribble)
        self.root = self.dribble_behavior
        self.root.setup_with_descendants()
        self.__name__ = 'dribble skill'

    def tick(self, robot: rc.Robot, world_state): #returns dict of robot and actions
        self.robot = robot
        actions = self.root.tick_once(self.robot, world_state)
        return actions

    def is_done(self, world_state):
        return self.move.is_done(world_state)
