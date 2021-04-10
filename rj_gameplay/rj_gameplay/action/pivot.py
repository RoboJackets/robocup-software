"""This module contains the interface and action for pivot."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.action as action
import numpy as np
import stp.rc as rc
from rj_msgs.msg import RobotIntent, PivotMotionCommand
from rj_geometry_msgs.msg import Point


class Pivot(IFiniteAction):
    """
    Pivot Skill
    """
    def __init__(self, robot_id : int, pivot_point: np.ndarray, target_point: np.ndarray, priority : int = 1):
        self.pivot_point = pivot_point
        self.target_point = target_point

    def tick(self, intent) -> None:
        pivot_command = PivotMotionCommand()
        pivot_command.pivot_point = Point(x=self.pivot_point[0], y=self.pivot_point=[1])
        intent.motion_command.pivot_motion_command.pivot_point = Point(x=)
        intent.active = True
        pass

    def is_done(self, world_state) -> bool:
        #vec = self.target_point
        #world_state.our_robots[self.robot_id].pose[2]
        return True
        
