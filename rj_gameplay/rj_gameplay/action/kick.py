"""This module contains the interface and action for kick."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.action as action
import stp.rc as rc
import numpy as np
from rj_msgs.msg import RobotIntent
# from rj_msgs.msg import RobotIntent, MotionCommand, EmptyMotionCommand, PathTargetMotionCommand, LinearMotionInstant
from rj_geometry_msgs.msg import Point
from typing import Optional
import math
from rj_msgs import msg

class Kick(action.IFiniteAction):
    """
    Activates kicker. Intended to be used after an aim/drive action in a skill.
    """
    def __init__(self, robot_id: int, priority: int = 0) -> None: 
        self.robot_id = robot_id
        self.priority = priority

        self.count = -1

    def tick(self, intent: msg.RobotIntent) -> msg.RobotIntent:
        self.count += 1
        # not sure if this is how you ref this constant (in RobotIntent.msg)
        intent.shoot_mode = intent.SHOOT_MODE_KICK 
        if self.count == 0:
            intent.kick_speed = 255 # kMaxKick is in rj_constants, not sure how to reference it
        elif self.count >= 1:
            intent.kick_speed = 0
        intent.is_active = True
        return intent 

    def is_done(self, world_state: rc.WorldState) -> bool:
        return self.count >= 2
