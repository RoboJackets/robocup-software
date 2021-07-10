"""This module contains the interface and action for intercept."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.action as action
import stp.rc as rc
import numpy as np
from rj_msgs.msg import RobotIntent, InterceptMotionCommand, SettleMotionCommand
from rj_geometry_msgs.msg import Point


class Intercept(action.IAction):
    """Intercept action
    """
    def __init__(self, robot_id: int = None, target_point: np.ndarray = None):
        self.robot_id = robot_id
        self.target_point = target_point

    def tick(self, intent: RobotIntent) -> None:
        """
        # TODO: use this when intercept is fixed
        intercept_command = InterceptMotionCommand()
        # TODO: numpy to Point conv
        intercept_command.target = Point(x=self.target_point[0], y=self.target_point[1])
        intent.motion_command.intercept_command = [intercept_command] 
        """
        settle_command = SettleMotionCommand()
        intent.motion_command.settle_command = [settle_command]
        intent.dribbler_speed = 1.0
        intent.is_active = True

        return intent

    def is_done(self, world_state) -> bool:
        if world_state.our_robots[self.robot_id].has_ball_sense:
            return True
        return False
