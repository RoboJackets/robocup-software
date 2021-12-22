from abc import ABC, abstractmethod

import rj_gameplay.eval as eval
import argparse
import py_trees
import sys
import time
from rj_msgs.msg import RobotIntent, SettleMotionCommand

import stp.skill as skill
import stp.role as role
import stp.rc as rc
import numpy as np
from typing import Optional

SETTLE_BALL_SPEED_THRESHOLD = 0.1
SETTLE_BALL_DIST_THRESHOLD = 0.1

class Settle(skill.ISkill):

    """
    Robot settles the ball as a receiver
    """

    #TODO: add move functionality so that robot can move to where the ball is going.

    def __init__(self, robot: rc.Robot = None):
        self.robot = robot
        
        self.__name__ = 'settle skill'


    def tick(self, 
             robot: rc.Robot, 
             world_state: rc.WorldState, 
             intent: RobotIntent):
        self.robot = robot
        settle_command = SettleMotionCommand()
        intent.motion_command.settle_command = [settle_command]
        intent.dribbler_speed = 1.0
        intent.trigger_mode = RobotIntent.TRIGGER_MODE_STAND_DOWN # 0
        intent.is_active = True

        # return intent
        return {self.robot.id: intent}


    def is_done(self, world_state) -> bool:
        if self.robot is None:
            return False

        robot_has_ball = world_state.our_robots[self.robot.id].has_ball_sense
        speed_is_slow = np.linalg.norm(world_state.ball.vel) < SETTLE_BALL_SPEED_THRESHOLD
        ball_is_close = np.linalg.norm(self.robot.pose[0:2] - world_state.ball.pos[0:2]) < SETTLE_BALL_DIST_THRESHOLD
        if robot_has_ball or (speed_is_slow and ball_is_close):
            return True
        return False


    # TODO: def __str__
