from abc import ABC, abstractmethod

import rj_gameplay.eval as eval
import argparse
import py_trees
import sys
import time

import stp.skill as skill
import stp.rc as rc
from rj_msgs.msg import RobotIntent, EmptyMotionCommand
import numpy as np

from stp.utils.constants import KickConstants

KICK_DOT_THRESHOLD = KickConstants.KICK_DOT_THRESHOLD
KICK_BALL_SPEED_THRESHOLD = KickConstants.KICK_BALL_SPEED_THRESHOLD


class Kick(skill.Skill):
    def __init__(
        self,
        robot: rc.Robot,
        chip: bool,
        kick_speed: float,
        threshold: float = 0.02,
    ) -> None:

        self.__name__ = "kick skill"
        self.robot = robot
        self.chip = chip
        self.kick_speed = kick_speed

    def tick(self, world_state: rc.WorldState) -> RobotIntent:
        super().tick(world_state)
        intent = RobotIntent()

        empty_command = EmptyMotionCommand()
        intent.motion_command.empty_command = [empty_command]
        intent.kick_speed = self.kick_speed
        intent.trigger_mode = 2
        intent.shoot_mode = self.chip
        intent.is_active = True

        return intent

    def is_done(self, world_state: rc.WorldState) -> bool:
        if self.robot is None:
            return False
        ball_vel_unit = world_state.ball.vel / np.linalg.norm(world_state.ball.vel)
        heading_angle = world_state.our_robots[self.robot.id].pose[2]
        heading_vect = np.array([np.cos(heading_angle), np.sin(heading_angle)])
        dot_product = np.dot(heading_vect, ball_vel_unit)
        # TODO: Make this threshold a local param
        if (
            dot_product > KICK_DOT_THRESHOLD
            and np.linalg.norm(world_state.ball.vel) > KICK_BALL_SPEED_THRESHOLD
        ):
            return True
        return False
