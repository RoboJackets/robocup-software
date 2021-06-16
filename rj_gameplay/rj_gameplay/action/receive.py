"""This module contains the interface and action for receive."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.action as action
import stp.rc as rc
import numpy as np
from rj_msgs.msg import RobotIntent, SettleMotionCommand

class Receive(action.IAction):
    """
    Receive action
    """

    def __init__(self, robot_id: int = None):
        self.robot_id = robot_id


    def tick(self, intent) -> None:
        settle_command = SettleMotionCommand()
        intent.motion_command.settle_command = [settle_command]
        intent.dribbler_speed = 1.0
        intent.is_active = True

        return intent

    def is_done(self, world_state) -> bool:
        #TODO: Use local params for this threshold
        if np.linalg.norm(world_state.ball.vel) < 0.005:
            return True
        return False