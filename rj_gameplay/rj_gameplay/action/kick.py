"""This module contains the interface and action for kick."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.action as action
import numpy as np
import stp.rc as rc
from typing import Optional
from rj_msgs.msg import RobotIntent, EmptyMotionCommand
from rj_geometry_msgs.msg import Point


class IKick(action.IAction, ABC):
    def done(self) -> bool:
        pass


class Kick(IKick):
    """
    Kick action
    """
    def __init__(self, robot_id:Optional[int]=None, chip:Optional[bool]=False, kick_speed:Optional[float]=255.0) -> None:
        #TODO: Change kick speed to use max_kick_speed param for default value
        self.robot_id = robot_id
        self.chip = chip
        self.kick_speed = kick_speed

    def tick(self, intent:RobotIntent) -> RobotIntent:
        new_intent = intent
        empty_command = EmptyMotionCommand()
        new_intent.motion_command.empty_command = [empty_command]
        intent.kick_speed = self.kick_speed
        new_intent.trigger_mode = 2
        new_intent.shoot_mode = self.chip
        new_intent.is_active = True
        return new_intent

    def is_done(self, world_state:rc.WorldState) -> bool:
        return np.linalg.norm(world_state.ball.vel) > 1
