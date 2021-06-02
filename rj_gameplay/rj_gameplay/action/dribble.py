"""This module contains the interface and action for dribble."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.action as action
import stp.rc as rc
from rj_msgs.msg import RobotIntent
from typing import Optional
from rj_msgs import msg
from rj_gameplay.action import move, activate_dribbler


class IDribble(action.IAction, ABC):

    def is_done(self) -> bool:
        pass

class Dribble(IDribble):
    """
    Dribble action
    """
    def __init__(self,
            robot_id : int,
            target_point : np.ndarray,
            target_vel : np.ndarray = np.array([0.0,0.0]),
            face_angle : Optional[float] = None,
            face_point : Optional[np.ndarray] = None,
            priority : int = 0) -> None:

        self.robot_id = robot_id
        self.target_point = target_point
        self.target_vel = target_vel
        self.face_angle = face_angle
        self.face_point = face_point
        self.priority = priority

    def tick(self, intent: msg.RobotIntent) -> msg.RobotIntent:
        # TODO: should both move and dribble at max speed
        intent.dribbler_speed = self.dribbler_speed
        intent.is_active = True
        return intent

    def is_done(self) -> bool:
        return False
