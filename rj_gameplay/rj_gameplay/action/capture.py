"""This module contains the interface and action for capture."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.action as action
import stp.rc as rc
import numpy as np
from rj_msgs.msg import RobotIntent, CollectMotionCommand

class Capture(action.IAction):
    """
    Capture action
    TODO: update with actions implementation
    """

    def __init__(self, robot_id: int = None):
        self.robot_id = robot_id


    def tick(self, intent) -> None:
        collect_command = CollectMotionCommand()
        intent.motion_command.collect_command = [collect_command] 
        intent.dribbler_speed = 1.0
        intent.active = True

        return intent

    def is_done(self, world_state) -> bool:
        if world_state.our_robots[self.robot_id].has_ball_sense:
            return True
        return False
