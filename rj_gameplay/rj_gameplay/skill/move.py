import argparse
import math
import sys
import time
from typing import Optional

import numpy as np
import stp.action as action
import stp.rc as rc
import stp.skill
from rj_geometry_msgs.msg import Point
from rj_msgs.msg import PathTargetMotionCommand, RobotIntent


class Move(stp.skill.Skill):
    def __init__(
        self,
        robot: Optional[rc.Robot] = None,
        target_point: np.ndarray = np.array([0.0, 0.0]),
        target_vel: np.ndarray = np.array([0.0, 0.0]),
        face_angle: float = None,
        face_point: np.ndarray = None,
        ignore_ball: bool = False,
        priority: int = 0,
    ):

        self.robot = robot
        self.target_point = target_point
        self.target_vel = target_vel
        self.face_point = face_point
        self.face_angle = face_angle
        self.ignore_ball = ignore_ball
        self.priority = priority

        self.cached_intent = None

        self.__name__ = "Move"

    def tick(self, world_state: rc.WorldState) -> RobotIntent:
        super().tick(world_state)
        intent = RobotIntent()
        path_command = PathTargetMotionCommand()
        path_command.target.position = Point(
            x=self.target_point[0], y=self.target_point[1]
        )
        path_command.target.velocity = Point(x=self.target_vel[0], y=self.target_vel[1])
        path_command.ignore_ball = self.ignore_ball

        if self.face_angle is not None:
            path_command.override_angle = [self.face_angle]

        if self.face_point is not None:
            path_command.override_face_point = [
                Point(x=self.face_point[0], y=self.face_point[1])
            ]

        intent.motion_command.path_target_command = [path_command]
        intent.is_active = True

        # TODO: motion planning is a lot more stable when not being spammed with repeat intents, use Action Client/Server to avoid re-requests when the intent is the same
        return intent

    def is_done(self, world_state: rc.WorldState) -> bool:
        position_tolerance = 1e-2
        velocity_tolerance = 1e-1
        if self.robot.id is None or world_state is None:
            return False

        # only linear for now (so this is technically not pose, which
        # includes angular position)
        robot_pt = world_state.our_robots[self.robot.id].pose[:2]
        goal_pt = self.target_point
        robot_vel = world_state.our_robots[self.robot.id].twist[:2]
        goal_vel = self.target_vel

        return (
            np.linalg.norm(robot_pt - goal_pt) < position_tolerance
            and np.linalg.norm(robot_vel - goal_vel) < velocity_tolerance
        )

    def __str__(self):
        ignore_ball_str = ", ignoring ball" if self.ignore_ball else ""
        return f"Move(robot={self.robot.id if self.robot is not None else '??'}, target={self.target_point}{ignore_ball_str})"

    def __repr__(self) -> str:
        return self.__str__()
