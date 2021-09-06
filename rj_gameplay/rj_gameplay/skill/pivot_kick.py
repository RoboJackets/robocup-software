from abc import ABC, abstractmethod

import rj_gameplay.eval as eval
import argparse
import py_trees
import sys
import time

import stp.skill as skill
import stp.role as role
from rj_gameplay.action import pivot, kick, capture
from stp.skill.action_behavior import ActionBehavior, RobotActions
from stp.skill.rj_sequence import RjSequence as Sequence
import stp.rc as rc
import numpy as np

MAX_DRIBBLER_SPEED = 1.0

class PivotKick(skill.ISkill): # add ABC if fails
    """
    A pivot kick skill
    capture -> pivot -> kick
    """

    def __init__(self,
                 robot: rc.Robot,
                 target_point: np.array,
                 chip: bool,
                 kick_speed: float,
                 threshold: float = 0.02) -> None:
        # TODO: Have something which automatically determines kick speed based on target point distance
        self.__name__ = 'pivot kick'
        self.robot = robot
        self.chip = chip
        self.kick_speed = kick_speed
        self.root = Sequence("Sequence")
        self.target_point = target_point

        if robot is not None:
            self.pivot = pivot.Pivot(robot.id, robot.pose[0:2], target_point,
                                     MAX_DRIBBLER_SPEED, threshold)
            self.kick = kick.Kick(self.robot.id, self.chip, self.kick_speed)
        else:
            self.pivot = pivot.Pivot(None, np.array([0.0, 0.0]), target_point,
                                     MAX_DRIBBLER_SPEED)
            self.kick = kick.Kick(None, self.chip, self.kick_speed)

        self.capture = capture.Capture()
        self.capture_behavior = ActionBehavior('Capture', self.capture)
        self.pivot_behavior = ActionBehavior('Pivot', self.pivot)
        self.kick_behavior = ActionBehavior('Kick', self.kick)
        self.root.add_children([self.capture_behavior, self.pivot_behavior, self.kick_behavior])
        self.root.setup_with_descendants()

    def tick(self, robot: rc.Robot, world_state: rc.WorldState) -> RobotActions:
        self.robot = robot
        self.pivot.robot_id = robot.id
        self.kick.robot_id = robot.id

        self.pivot.pivot_point = world_state.ball.pos
        self.pivot.target_point = self.target_point
        actions = self.root.tick_once(robot, world_state)
        return actions

    def is_done(self, world_state: rc.WorldState) -> bool:
<<<<<<< HEAD
        return self.pivot.is_done(world_state) and self.kick.is_done(world_state)
=======
        return self.pivot.is_done(world_state) and self.kick.is_done(
            world_state)

    def __str__(self):
        return f"Pivot(robot={self.robot.id if self.robot is not None else '??'}, target={self.target_point})"
>>>>>>> 19154d927b2b8fa91e7749eb7539d8edf2f284b0
