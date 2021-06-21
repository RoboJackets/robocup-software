"""This module contains the interface and action for receive."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.action as action
import stp.rc as rc
import numpy as np
from rj_msgs.msg import RobotIntent, SettleMotionCommand

SETTLE_BALL_SPEED_THRESHOLD = 1.0

class Receive(action.IAction):
    """
    Receive action
    """

    def __init__(self, robot_id: int = None):
        self.robot_id = robot_id
        self.has_ball_ticks = 0


    def tick(self, intent) -> None:
        settle_command = SettleMotionCommand()
        intent.motion_command.settle_command = [settle_command]
        intent.dribbler_speed = 1.0
        intent.is_active = True

        return intent

    def is_done(self, world_state) -> bool:
        if self.robot_id is None:
            return False
        if world_state.our_robots[self.robot_id].has_ball_sense:
            self.has_ball_ticks += 1
        else:
            self.has_ball_ticks = 0
        #TODO: Use local params for this threshold
        if self.has_ball_ticks >= 15 or np.linalg.norm(world_state.ball.vel) < SETTLE_BALL_SPEED_THRESHOLD:
            return True
        return False
