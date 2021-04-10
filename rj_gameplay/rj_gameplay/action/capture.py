"""This module contains the interface and action for capture."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.action as action
import numpy as np
from rj_msgs.msg import RobotIntent


class Capture(ICapture):
    """
    Capture action
    TODO: update with actions implementation
    """

    def __init__(self, robot_id):
        self.robot_id = robot_id

    def tick(self, intent) -> None:
        

    def is_done(self) -> bool:
        pass
        
