import argparse
import sys
import time
from abc import ABC, abstractmethod
from typing import Optional

import numpy as np
import py_trees
import stp.action as action
import stp.rc as rc
import stp.role as role
import stp.skill as skill
from rj_msgs.msg import CollectMotionCommand, RobotIntent
from stp.utils.constants import BallConstants, RobotConstants

import rj_gameplay.eval as eval


class Capture(skill.Skill):
    """Second half of a Receive Skill. Captures the ball once it has been slowed down."""

    def __init__(self, robot: Optional[rc.Robot] = None):
        self.robot = robot
        self.ticks_done = 0

        self.__name__ = "capture skill"

    def tick(self, world_state: rc.WorldState) -> RobotIntent:
        super().tick(world_state)
        intent = RobotIntent()

        collect_command = CollectMotionCommand()
        intent.motion_command.collect_command = [collect_command]
        intent.dribbler_speed = 1.0  # dribbler is on by default
        intent.is_active = True

        return intent

    def is_done(self, world_state) -> bool:
        ball_speed = np.linalg.norm(world_state.ball.vel)

        ball_pos = world_state.ball.pos

        robot_pos = self.robot.pose[0:2]

        dist_to_ball = np.linalg.norm(robot_pos - ball_pos)

        ball_slow = ball_speed < 0.05
        ball_close = (
            dist_to_ball - (RobotConstants.RADIUS + BallConstants.RADIUS) < 0.03
        )

        return ball_slow and ball_close

        # TODO: has_ball_sense is broken, fix or wait for actionclient?
        # if (
        #     self.robot is not None
        #     and world_state.our_robots[self.robot.id].has_ball_sense
        # ):
        #     self.ticks_done += 1
        # else:
        #     self.ticks_done -= 5
        # self.ticks_done = np.clip(self.ticks_done, a_min=0, a_max=200)
        # return self.ticks_done > 50

    def __str__(self):
        return f"Capture(robot={self.robot.id if self.robot is not None else '??'})"

    def __repr__(self) -> str:
        return self.__str__()
