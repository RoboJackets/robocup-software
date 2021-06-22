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


class LineKickAction(action.IFiniteAction):
    """
    Activates kicker. Intended to be used after an aim/drive action in a skill.
    """

    def __init__(self, robot_id: int, target: np.ndarray, priority: int = 0) -> None:
        self.robot_id = robot_id
        self.priority = priority

        self.target = target

    def tick(self, intent: msg.RobotIntent) -> msg.RobotIntent:
        line_kick_command = LineKickMotionCommand()
        line_kick_command.target = Point(x=self.target[0], y=self.target[1])
        intent.shoot_mode = RobotIntent.SHOOT_MODE_KICK
        intent.trigger_mode = RobotIntent.TRIGGER_MODE_ON_BREAK_BEAM
        intent.kick_speed = 6.0

        intent.motion_command.line_kick_command = [line_kick_command]
        intent.is_active = True
        return intent

    def is_done(self, world_state: rc.WorldState) -> bool:
        return np.linalg.norm(world_state.ball.vel) > 1.0
