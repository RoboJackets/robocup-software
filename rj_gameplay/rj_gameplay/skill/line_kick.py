import argparse
import sys
import time
from abc import ABC
from enum import Enum, auto

import numpy as np
import stp.action as action
import stp.rc as rc
import stp.skill as skill
from rj_geometry_msgs.msg import Point
from rj_msgs.msg import LineKickMotionCommand, RobotIntent


class State(Enum):
    CAPTURE = auto()
    PIVOT = auto()
    KICK = auto()
    DONE = auto()


class LineKick(skill.Skill):
    """
    A skill version of line kick so that actions don't have to be called in tactics
    """

    # role-based implementation
    # def __init__(self, role: role.Role) -> None:
    # self.robot = role.robot
    # role-blind implementation
    def __init__(
        self,
        robot: rc.Robot,
        target_point: np.array,
        priority: int = 0,
        chip: bool = False,
        kick_speed: float = 5.5,
    ) -> None:
        self.robot = robot

        self.target_point = target_point
        self.priority = priority
        self.chip = chip
        self.kick_speed = kick_speed

    def tick(self, world_state: rc.WorldState) -> RobotIntent:
        super().tick(world_state)

        intent = RobotIntent()

        ball_to_target = self.target_point - world_state.ball.pos
        ball_to_target /= np.linalg.norm(ball_to_target)
        robot_dir = np.array([np.cos(self.robot.pose[2]), np.sin(self.robot.pose[2])])

        right_direction = np.dot(ball_to_target, robot_dir) > 0.9
        if not right_direction:
            self.kick_speed = 0.0

        line_kick_command = LineKickMotionCommand()
        line_kick_command.target = Point(x=self.target_point[0], y=self.target_point[1])
        intent.shoot_mode = (
            RobotIntent.SHOOT_MODE_KICK
            if not self.chip
            else RobotIntent.SHOOT_MODE_CHIP
        )
        intent.trigger_mode = RobotIntent.TRIGGER_MODE_ON_BREAK_BEAM
        intent.kick_speed = self.kick_speed

        intent.motion_command.line_kick_command = [line_kick_command]
        intent.is_active = True

        return intent

    def is_done(self, world_state: rc.WorldState):
        # copy-pasted from kick skill
        KICK_DOT_THRESHOLD = 0.4
        KICK_BALL_SPEED_THRESHOLD = 0.9

        # TODO: make pivot kick and line kick inherit from some common kick superclass to make this cleaner
        if self.robot is None:
            return False
        ball_vel_unit = world_state.ball.vel / np.linalg.norm(world_state.ball.vel)
        heading_angle = world_state.our_robots[self.robot.id].pose[2]
        heading_vect = np.array([np.cos(heading_angle), np.sin(heading_angle)])
        dot_product = np.dot(heading_vect, ball_vel_unit)
        # TODO: Make this threshold a local param
        ball_too_far = np.linalg.norm(world_state.ball.pos - self.robot.pose[0:2]) > 0.5
        if ball_too_far or (
            dot_product > KICK_DOT_THRESHOLD
            and np.linalg.norm(world_state.ball.vel) > KICK_BALL_SPEED_THRESHOLD
        ):
            return True
        return False

    def __str__(self):
        return f"LineKick(robot={self.robot.id if self.robot is not None else '??'}, target={self.target_point}, chip={self.chip}, kick_speed={self.kick_speed})"

    def __repr__(self) -> str:
        return self.__str__()
