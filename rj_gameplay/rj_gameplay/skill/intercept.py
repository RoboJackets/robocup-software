from abc import ABC, abstractmethod

import rj_gameplay.eval as eval
import argparse
import py_trees
import sys
import time
import numpy as np
from rj_msgs.msg import RobotIntent, PathTargetMotionCommand
from rj_geometry_msgs.msg import Point

import stp.skill as skill
import stp.role as role
import stp.rc as rc
from typing import Optional

SETTLE_BALL_SPEED_THRESHOLD = 1.0
INTERCEPT_ANGLE_THRESHOLD = 2*np.pi / 36


class Intercept(skill.Skill):
    """First third of a PassReceive Skill. Gets in front of moving ball."""
    """TODO: Replace with actual intercept planner"""

    def __init__(self, robot: rc.Robot = None):
        self.robot = robot

        self.__name__ = "intercept skill"

    def get_intercept_pt(self, world_state: rc.WorldState, my_robot: np.ndarray) -> np.ndarray:
        pos = world_state.ball.pos
        vel = world_state.ball.vel

        ball_dir = vel / (np.linalg.norm(vel) + 1e-6)

        ball_to_bot = pos - my_robot.pose[:1]
        # ball_to_bot_unt = ball_to_bot / (np.linalg.norm(ball_to_bot) + 1e-6)
        intercept_pt = np.dot(ball_to_bot, ball_dir)*ball_dir

        return intercept_pt


    def tick(self, world_state: rc.WorldState) -> RobotIntent:
        super().tick(world_state)
        intent = RobotIntent()
        path_command = PathTargetMotionCommand()
        target_point = self.get_intercept_pt(world_state, self.robot)
        target_vel = [0.0, 0.0]
        path_command.target.position = Point(
            x=target_point[0], y=target_point[1]
        )
        path_command.target.velocity = Point(x=target_vel[0], y=target_vel[1])
        path_command.ignore_ball = False

        path_command.override_face_point = [
            Point(x=world_state.ball.pos[0], y=world_state.ball.pos[1])
        ]

        intent.motion_command.path_target_command = [path_command]
        intent.is_active = True

        # TODO: motion planning is a lot more stable when not being spammed with repeat intents, use Action Client/Server to avoid re-requests when the intent is the same
        return intent

    def is_done(self, world_state) -> bool:
        if self.robot is None:
            return False

        ball_dir = world_state.ball.vel / (np.linalg.norm(world_state.ball.vel) + 1e-6)
        ball_to_bot = world_state.ball.pos - self.robot.pose[:1]
        ball_to_bot_unt = ball_to_bot / (np.linalg.norm(ball_to_bot) + 1e-6)
        dot_prod = np.dot(ball_to_bot_unt, ball_dir)
        angle = np.arccos(dot_prod)
        if (
            world_state.our_robots[self.robot.id].has_ball_sense
            or np.linalg.norm(world_state.ball.vel) < SETTLE_BALL_SPEED_THRESHOLD
            or angle < INTERCEPT_ANGLE_THRESHOLD
        ):
            return True
        return False

    def __str__(self):
        return f"Intercept(robot={self.robot.id if self.robot is not None else '??'})"

    def __repr__(self) -> str:
        return self.__str__()
