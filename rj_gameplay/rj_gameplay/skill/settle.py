from abc import ABC, abstractmethod

import rj_gameplay.eval as eval
import argparse
import py_trees
import sys
import time
from rj_msgs.msg import RobotIntent, SettleMotionCommand, PivotMotionCommand
from rj_geometry_msgs.msg import Point

import stp.skill as skill
import stp.role as role
import stp.rc as rc
from typing import Optional
import numpy as np
from rj_gameplay.tactic import pass_tactic

from rj_gameplay.MAX_KICK_SPEED import *

SETTLE_BALL_SPEED_THRESHOLD = 0.8


class Settle(skill.ISkill):

    """
    # Robot settles the ball as a receiver
    """

    #TODO: add move functionality so that robot can move to where the ball is going.

    def __init__(self,
                 robot: rc.Robot,
                 chip: bool = False,
                 kick_speed: float = MAX_KICK_SPEED,
                 threshold: float = 0.02):
        self.robot = robot
        self.passer = None

        self.__name__ = 'settle skill'


    def tick(self, robot: rc.Robot, world_state: rc.WorldState,
             intent: RobotIntent):
        self.robot = robot
        self.passer = pass_tactic.Pass().find_passer(world_state)

        settle_command = SettleMotionCommand()
        intent.motion_command.settle_command = [settle_command]
        intent.dribbler_speed = 1.0
        intent.is_active = True

        return {self.robot.id: intent}


    def is_done(self, world_state) -> bool:
        if self.robot is None:
            return False
        if world_state.our_robots[self.robot.id].has_ball_sense or (
                np.linalg.norm(self.robot.pose[0:2] - world_state.ball.pos[0:2]
                               ) < np.linalg.norm(self.passer.pose[0:2] -
                                                  world_state.ball.pos[0:2])
                and np.linalg.norm(
                    world_state.ball.vel) < SETTLE_BALL_SPEED_THRESHOLD):
            return True

        return False


    # TODO: def __str__
