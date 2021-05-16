"""This module contains the interface and action for capture."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.action as action
import stp.rc as rc
import numpy as np
from rj_msgs.msg import RobotIntent, SettleMotionCommand

class Capture(action.IAction):
    """
    Capture action
    TODO: update with actions implementation
    """

    def __init__(self, robot_id):
        self.robot_id = robot_id


    def tick(self, intent) -> None:
        settle_command = SettleMotionCommand()
        intent.motion_command.settle_command = [settle_command] 
        intent.dribbler_speed = 1.0
        intent.active = True

        return intent

    def is_done(self, world_state) -> bool:
        if world_state.our_robots[self.robot_id].has_ball_sense:
            return True
        return False