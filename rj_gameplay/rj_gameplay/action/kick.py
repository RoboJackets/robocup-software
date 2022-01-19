"""This module contains the interface and action for kick."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.action as action
import numpy as np
import stp.rc as rc
from typing import Optional
from rj_msgs.msg import RobotIntent, EmptyMotionCommand
from rj_geometry_msgs.msg import Point

KICK_DOT_THRESHOLD = 0.4
KICK_BALL_SPEED_THRESHOLD = 0.9


class Kick(action.IAction):  # add ABC if needed
    """
    Kick action
    """

    def __init__(self, robot_id: int, chip: bool, kick_speed: float) -> None:
        self.robot_id = robot_id
        self.chip = chip
        self.kick_speed = kick_speed

    def tick(self, intent: RobotIntent) -> RobotIntent:
        new_intent = intent
        empty_command = EmptyMotionCommand()
        new_intent.motion_command.empty_command = [empty_command]
        new_intent.trigger_mode = 2
        new_intent.shoot_mode = self.chip
        new_intent.is_active = True
        return new_intent

    def is_done(self, world_state: rc.WorldState) -> bool:
        if self.robot_id is None:
            return False
        ball_vel_unit = world_state.ball.vel / np.linalg.norm(world_state.ball.vel)
        heading_angle = world_state.our_robots[self.robot_id].pose[2]
        heading_vect = np.array([np.cos(heading_angle), np.sin(heading_angle)])
        dot_product = np.dot(heading_vect, ball_vel_unit)
        # TODO: Make this threshold a local param
        if (
            dot_product > KICK_DOT_THRESHOLD
            and np.linalg.norm(world_state.ball.vel) > KICK_BALL_SPEED_THRESHOLD
        ):
            return True
        return False
