"""This module contains the interface and action for pivot."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.action as action
import numpy as np
import stp.rc as rc
from rj_msgs.msg import RobotIntent, PivotMotionCommand
from rj_geometry_msgs.msg import Point



class Pivot(action.IFiniteAction):

    def __init__(self, robot_id : int, pivot_point: np.ndarray, target_point: np.ndarray, dribble_speed : float = 0, priority : int = 1):
        self.pivot_point = pivot_point
        self.target_point = target_point
        self.dribble_speed =dribble_speed

    def tick(self, intent: msg.RobotIntent) -> None:
        pivot_command = PivotMotionCommand()
        pivot_command.pivot_point = Point(x=self.pivot_point[0], y=self.pivot_point[1])
        pivot_command.target_point = Point(x=self.target_point[0], y=self.target_point[1])
        intent.motion_command.pivot_motion_command = [pivot_command]
        intent.dribbler_speed = self.drible_speed
        intent.active = True
        return intent

    def is_done(self, world_state) -> bool:
        #vec = self.target_point
        #world_state.our_robots[self.robot_id].pose[2]
        return False