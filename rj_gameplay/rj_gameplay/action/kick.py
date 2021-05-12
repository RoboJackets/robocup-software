"""This module contains the interface and action for kick."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.action as action
import stp.rc as rc
import numpy as np
# from rj_msgs.msg import RobotIntent, MotionCommand, EmptyMotionCommand, PathTargetMotionCommand, LinearMotionInstant
from rj_geometry_msgs.msg import Point
from typing import Optional
import math
from rj_msgs import msg


class IKick(action.IAction, ABC):
    def done(self) -> bool:
        pass


class Kick(IKick):
    """
    Kick action
    TODO: update with actions implementation
    """
    def __init__(self, 
            robot_id : int, 
            target_point : np.ndarray):

        self.robot_id = robot_id
        self.target_point = target_point 

        self.count = -1

    def tick(self, intent: msg.RobotIntent) -> msg.RobotIntent:
        # print('robot:', self.robot_id, 'kicking')
        intent.shoot_mode = intent.SHOOT_MODE_KICK # not sure if this is how you ref that constant in RobotIntent.msg
        intent.kick_speed = kMaxKick #255
        intent.is_active = True

        self.count += 1

    def done(self) -> bool:
        return self.count == 1

    def fail(self):
        return False
