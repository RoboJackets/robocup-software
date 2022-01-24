from abc import ABC, abstractmethod

import rj_gameplay.eval as eval
import argparse
import py_trees
import sys
import time

import stp.skill as skill
import stp.role as role
from rj_msgs.msg import RobotIntent, PivotMotionCommand
import stp.rc as rc
import numpy as np
from rj_geometry_msgs.msg import Point


class Pivot(skill.Skill):
    def __init__(
        self,
        robot: rc.Robot = None,
        pivot_point: np.ndarray = None,
        target_point: np.ndarray = None,
        dribble_speed: float = 1.0,
        threshold: float = 0.02,
        priority: int = 1,
    ):
        self.robot = robot
        self.pivot_point = pivot_point
        self.target_point = target_point
        self.dribble_speed = dribble_speed
        self.threshold = threshold

        self.__name__ = "pivot skill"

    def tick(self, world_state: rc.WorldState) -> RobotIntent:
        super().tick(world_state=world_state)
        intent = RobotIntent()

        self.pivot_point = world_state.ball.pos

        pivot_command = PivotMotionCommand()
        pivot_command.pivot_point = Point(x=self.pivot_point[0], y=self.pivot_point[1])
        pivot_command.pivot_target = Point(
            x=self.target_point[0], y=self.target_point[1]
        )
        intent.motion_command.pivot_command = [pivot_command]
        intent.dribbler_speed = self.dribble_speed
        intent.is_active = True

        return intent

    def is_done(self, world_state: rc.WorldState) -> bool:
        if self.robot is None:
            return False
        angle_threshold = self.threshold
        stopped_threshold = (
            5 * self.threshold
        )  # We don't _really_ care about this when we're kicking, if not for latency
        robot = world_state.our_robots[self.robot.id]
        robot_pos_to_target = self.target_point - robot.pose[0:2]
        robot_to_target_unit = robot_pos_to_target / np.linalg.norm(robot_pos_to_target)
        heading_vect = np.array([np.cos(robot.pose[2]), np.sin(robot.pose[2])])
        dot_product = np.dot(heading_vect, robot_to_target_unit)
        angle = np.arccos(dot_product)
        if (angle < angle_threshold) and (
            abs(world_state.our_robots[self.robot.id].twist[2]) < stopped_threshold
        ):
            return True
        else:
            return False
