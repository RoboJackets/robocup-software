"""This module contains the interface and action for activate the dribbler."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.action as action
import stp.rc as rc
from rj_msgs.msg import RobotIntent
from typing import Optional
from rj_msgs import msg


class ActivateDribbler(action.IAction):  # add ABC if this fails
    """
    Activate dribbler action
    """

    def __init__(
        self, robot_id: int, dribbler_speed: float = 1.0, priority: int = 0
    ) -> None:

        self.robot_id = robot_id
        self.dribbler_speed = dribbler_speed

    def tick(self, intent: msg.RobotIntent) -> msg.RobotIntent:
        intent.dribbler_speed = self.dribbler_speed
        intent.is_active = True
        return intent

    def is_done(self) -> bool:
        return False
