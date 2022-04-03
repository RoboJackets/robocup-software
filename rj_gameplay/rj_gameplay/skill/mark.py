import argparse
import math
import sys
import time
from abc import ABC, abstractmethod
from typing import Callable, Optional

import numpy as np
import py_trees
import stp.action as action
import stp.rc as rc
import stp.role as role
import stp.skill as skill
from stp.utils.constants import RobotConstants

import rj_gameplay.eval as eval
from rj_geometry_msgs.msg import Point, Segment
from rj_msgs.msg import PathTargetMotionCommand, RobotIntent


def get_mark_point(target_robot_id: int, world_state: rc.WorldState):
    # workaround for non-working CostBehavior:
    # initialize move action, update target point every tick (target point being opponent robot pos)

    # TODO: use mark_heuristic & CostBehavior to handle marking
    # argument: mark_heuristic: Callable[[np.array], float]
    # > self.mark_heuristic = mark_heuristic

    # dist away from target_robot to mark
    # TODO: add to global param server
    SAG_DIST = RobotConstants.RADIUS * 0.5

    # find point between ball and target robot that leaves SAG_DIST between edges of robots
    # this will be mark point
    ball_pos = world_state.ball.pos
    opp_pos = world_state.their_robots[target_robot_id].pose[0:2]

    mark_dir = (ball_pos - opp_pos) / np.linalg.norm(ball_pos - opp_pos)
    mark_pos = opp_pos + mark_dir * (2 * RobotConstants.RADIUS + SAG_DIST)

    # if ball inside robot radius of mark_pos, can't mark normally
    if np.linalg.norm(mark_pos - ball_pos) < RobotConstants.RADIUS:
        # instead, get in front of opp robot holding ball
        mark_pos += mark_dir * 2 * RobotConstants.RADIUS

    return mark_pos


"""
A skill which marks a given opponent robot according to some heuristic cost function
"""


# TODO: delete mark skill -> change to tactic
class Mark(skill.ISkill):
    def __init__(
        self,
        robot: rc.Robot = None,
        target_robot: rc.Robot = None,
        face_point: np.ndarray = None,
        face_angle: Optional[float] = None,
        target_vel: np.ndarray = np.array([0.0, 0.0]),
        ignore_ball: bool = False,
    ):

        self.__name__ = "Mark"
        self.robot = robot
        self.target_robot = target_robot
        self.target_vel = target_vel
        self.face_point = face_point
        self.face_angle = face_angle
        self.ignore_ball = ignore_ball

    def tick(self, world_state: rc.WorldState):
        intent = RobotIntent()
        mark_point = np.array([0.0, 0.0])
        if world_state and world_state.ball.visible:
            if self.target_robot is None:
                mark_point = get_mark_point(1, world_state)
            else:
                mark_point = get_mark_point(self.target_robot.id, world_state)
        self.target_point = mark_point
        self.face_point = world_state.ball.pos

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

    def __str__(self):
        return f"Mark(robot={self.robot.id if self.robot is not None else '??'}, target={self.target_robot.id if self.target_robot is not None else '??'})"

    def __repr__(self) -> str:
        return self.__str__()
