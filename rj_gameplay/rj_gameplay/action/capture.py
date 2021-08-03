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
    """

    def __init__(self, robot_id: int = None):
        self.robot_id = robot_id
        self.ticks_done = 0

    def tick(self, intent) -> None:
        collect_command = CollectMotionCommand()
        intent.motion_command.collect_command = [collect_command]
        intent.dribbler_speed = 1.0
        intent.is_active = True
        return intent

    def is_done(self, world_state) -> bool:
        if self.robot_id is not None and world_state.our_robots[
                self.robot_id].has_ball_sense:
            self.ticks_done += 1
        else:
            self.ticks_done -= 5
        self.ticks_done = np.clip(self.ticks_done, a_min=0, a_max=200)
        return self.ticks_done > 50
