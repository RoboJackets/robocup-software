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
from rj_gameplay.action import move
from stp.skill.action_behavior import ActionBehavior
import stp.rc as rc
from rj_msgs import msg

class IMove(skill.ISkill, ABC):
    ...


"""
A skill version of move so that actions don't have to be called in tactics
"""
class Move(IMove):
    def __init__(self,
            robot : rc.Robot = None,
            target_point : np.ndarray = np.array([0.0,0.0]),
            target_vel : np.ndarray = np.array([0.0,0.0]),
            face_angle : Optional[float] = None,
            face_point : Optional[np.ndarray] = None,
            ignore_ball: bool = False):
        self.robot = robot
        self.target_point = target_point
        self.target_vel = target_vel
        self.face_point = face_point
        self.face_angle = face_angle
        self.ignore_ball = ignore_ball

        if self.robot is not None:
            self.move = move.Move(self.robot.id, target_point, target_vel, face_angle, face_point, priority=0, ignore_ball=ignore_ball)
        else:
            self.move = move.Move(self.robot, target_point, target_vel, face_angle, face_point, priority=0, ignore_ball=ignore_ball)

        self.move_behavior = ActionBehavior('Move', self.move)
        self.root = self.move_behavior
        self.root.setup_with_descendants()
        self.__name__ = 'move skill'

    def tick(self, robot: rc.Robot,
             world_state: rc.WorldState):  #returns dict of robot and actions
        self.robot = robot
        self.move.target_point = self.target_point
        self.move.target_vel = self.target_vel
        self.move.face_angle = self.face_angle
        self.move.face_point = self.face_point
        actions = self.root.tick_once(self.robot, world_state)
        return actions

    def is_done(self, world_state):
        return self.move.is_done(world_state)

    def __str__(self):
        ignore_ball_str = ', ignoring ball' if self.ignore_ball else ''
        return f"Move(robot={self.robot.id if self.robot is not None else '??'}, target={self.target_point}{ignore_ball_str})"
