import argparse
import math
import sys
import time
from typing import Optional, Tuple

import numpy as np
import stp.action as action
import stp.rc as rc
import stp.skill as skill
from rj_geometry_msgs.msg import Point
from rj_msgs.msg import PathTargetMotionCommand, RobotIntent
from stp.utils.constants import RobotConstants


def get_mark_point(face_point: np.ndarray, block_point: np.ndarray):
    """
    face_point and block_point are the two points robot will mark between
    face_point: robot faces this way
    block_point: robot blocks this point from face_point
    """

    # dist away from target_robot to mark
    # TODO: add to global param server
    SAG_DIST = RobotConstants.RADIUS * 0.5

    mark_dir = (face_point - block_point) / (np.linalg.norm(face_point - block_point) + 1e-6)
    mark_pos = block_point + mark_dir * (2 * RobotConstants.RADIUS + SAG_DIST)

    # if ball inside robot radius of mark_pos, can't mark normally
    if np.linalg.norm(mark_pos - face_point) < RobotConstants.RADIUS:
        # instead, get in the way of mark pos
        mark_pos += mark_dir * 2 * RobotConstants.RADIUS

    return mark_pos


"""
A skill which marks a given opponent robot according to some heuristic cost function
"""


class Mark(skill.Skill):
    def __init__(
        self,
        robot: rc.Robot,
        face_point: np.ndarray,
        block_point: np.ndarray,
        ignore_ball: bool = False,
    ):
        self.__name__ = "Mark"
        self.robot = robot
        self.face_point = face_point
        self.block_point = block_point
        self.ignore_ball = ignore_ball

    def tick(self, world_state: rc.WorldState):
        super().tick(world_state)

        intent = RobotIntent()
        self.target_point = get_mark_point(self.face_point, self.block_point)

        path_command = PathTargetMotionCommand()
        path_command.target.position = Point(
            x=self.target_point[0], y=self.target_point[1]
        )
        path_command.target.velocity = Point(x=0.0, y=0.0)
        path_command.ignore_ball = self.ignore_ball
        path_command.override_face_point = [
            Point(x=self.face_point[0], y=self.face_point[1])
        ]

        intent.motion_command.path_target_command = [path_command]
        intent.is_active = True
        return intent
        # update target point every tick to match movement of ball & target robot

    def is_done(self, world_state) -> bool:
        threshold = 0.3
        if self.robot.id is None or world_state is None:
            return False
        elif (
            math.sqrt(
                (world_state.our_robots[self.robot.id].pose[0] - self.target_point[0])
                ** 2
                + (world_state.our_robots[self.robot.id].pose[1] - self.target_point[1])
                ** 2
            )
            < threshold
        ):
            return True
        else:
            return False

    def __repr__(self):
        return f"MarkSkill(face_point: {self.face_point}, block_point: {self.block_point})"
