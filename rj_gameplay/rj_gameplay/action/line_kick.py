"""This module contains the interface and action for kick."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.action as action
import stp.rc as rc
import numpy as np
from rj_msgs.msg import RobotIntent, MotionCommand, EmptyMotionCommand, LineKickMotionCommand
from rj_geometry_msgs.msg import Point
from typing import Optional
import math
from rj_msgs import msg

from rj_gameplay.MAX_KICK_SPEED import *


class LineKickAction(action.IFiniteAction):
    """
    Activates kicker. Intended to be used after an aim/drive action in a skill.
    """
    def __init__(self,
                 robot_id: int,
                 target: np.ndarray,
                 priority: int = 0,
                 chip: bool = False,
                 kick_speed: float = 6.0) -> None:
        self.robot_id = robot_id
        self.priority = priority

        self.target = target
        self.chip = chip
        self.kick_speed = kick_speed

    def tick(self, intent: msg.RobotIntent) -> msg.RobotIntent:
        line_kick_command = LineKickMotionCommand()
        line_kick_command.target = Point(x=self.target[0], y=self.target[1])
        intent.shoot_mode = RobotIntent.SHOOT_MODE_KICK if not self.chip else RobotIntent.SHOOT_MODE_CHIP
        intent.trigger_mode = RobotIntent.TRIGGER_MODE_ON_BREAK_BEAM
        if self.kick_speed <= MAX_KICK_SPEED:
            intent.kick_speed = self.kick_speed
        else:
            intent.kick_speed = MAX_KICK_SPEED

        intent.motion_command.line_kick_command = [line_kick_command]
        intent.is_active = True
        return intent

    def is_done(self, world_state: rc.WorldState) -> bool:
        return np.linalg.norm(world_state.ball.vel) > 1.0
