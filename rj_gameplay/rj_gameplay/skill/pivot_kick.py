from abc import ABC, abstractmethod

import rj_gameplay.eval as eval
import argparse
import py_trees
import sys
import time

import stp.skill as skill
import stp.role as role
import rj_gameplay.action as action
from stp.skill.action_behavior import ActionBehavior, RobotActions
from stp.skill.rj_sequence import RjSequence as Sequence
import stp.rc as rc
import numpy as np

MAX_DRIBBLER_SPEED = 1.0

class IPivotKick(skill.ISkill, ABC):
    ...

class PivotKick(IPivotKick):
    """
    A pivot kick skill
    """

    def __init__(self, robot: rc.Robot, target_point: np.array, chip: bool, kick_speed: float) -> None:
        #TODO: Have something which automatically determines kick speed based on target point distance
        self.__name__ = 'pivot kick'
        self.robot = robot
        self.chip = chip
        self.kick_speed = kick_speed
        self.root = Sequence("Sequence")
        self.target_point = target_point
        if robot is not None:
            self.pivot = action.pivot.Pivot(robot.id ,robot.pose[0:2], target_point, MAX_DRIBBLER_SPEED)
            self.kick = action.kick.Kick(self.robot.id, self.chip, self.kick_speed)
        else:
            self.pivot = action.pivot.Pivot(None, np.array([0.0,0.0]), target_point, MAX_DRIBBLER_SPEED)
            self.kick = action.kick.Kick(self.robot, self.chip, self.kick_speed)
        self.capture = action.capture.Capture()
        self.capture_behavior = ActionBehavior('Capture', self.capture)
        self.pivot_behavior = ActionBehavior('Pivot', self.pivot) 
        self.kick_behavior = ActionBehavior('Kick', self.kick)
        self.root.add_children([self.capture_behavior, self.pivot_behavior, self.kick_behavior])
        self.root.setup_with_descendants()

    def tick(self, robot: rc.Robot, world_state: rc.WorldState) -> RobotActions:
        self.robot = robot
        self.pivot.pivot_point = world_state.ball.pos
        self.pivot.target_point = self.target_point
        actions = self.root.tick_once(robot, world_state)
        return actions

    def is_done(self, world_state: rc.WorldState) -> bool:
        return self.pivot.is_done(world_state) and self.kick.is_done(world_state)