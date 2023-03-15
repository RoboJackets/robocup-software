import argparse
import sys
import time

import numpy as np
import stp.rc as rc
import stp.skill as skill
from rj_msgs.msg import EmptyMotionCommand, RobotIntent

KICK_DOT_THRESHOLD = 0.4
KICK_BALL_SPEED_THRESHOLD = 0.9


class Kick(skill.Skill):
    def __init__(
        self,
        robot: rc.Robot,
        chip: bool,
        kick_speed: float,
        threshold: float = 0.02,
    ) -> None:
        self.__name__ = "kick skill"
        self.robot = robot
        self.chip = chip
        self.kick_speed = kick_speed

    def tick(self, world_state: rc.WorldState) -> RobotIntent:
        super().tick(world_state)
        intent = RobotIntent()

        empty_command = EmptyMotionCommand()
        intent.motion_command.empty_command = [empty_command]
        intent.kick_speed = self.kick_speed
        intent.is_active = True

        # no chipping for now (4/18/22)
        # intent.shoot_mode = RobotIntent.SHOOT_MODE_KICK if not self.chip else RobotIntent.SHOOT_MODE_CHIP

        # config 1: works ~30% of the time
        intent.dribbler_speed = 1.0
        intent.shoot_mode = RobotIntent.SHOOT_MODE_KICK
        intent.trigger_mode = RobotIntent.TRIGGER_MODE_ON_BREAK_BEAM

        # config 2: about the same as 1
        # intent.dribbler_speed = 0.0
        # intent.shoot_mode = RobotIntent.SHOOT_MODE_KICK
        # intent.trigger_mode = RobotIntent.TRIGGER_MODE_IMMEDIATE
        # config 3: they're all the same...
        # intent.dribbler_speed = 0.5
        # intent.shoot_mode = RobotIntent.SHOOT_MODE_KICK
        # intent.trigger_mode = RobotIntent.TRIGGER_MODE_ON_BREAK_BEAM

        return intent

    def is_done(self, world_state: rc.WorldState) -> bool:
        if self.robot is None:
            return False
        ball_vel_unit = world_state.ball.vel / np.linalg.norm(world_state.ball.vel)
        heading_angle = world_state.our_robots[self.robot.id].pose[2]
        heading_vect = np.array([np.cos(heading_angle), np.sin(heading_angle)])
        dot_product = np.dot(heading_vect, ball_vel_unit)

        # TODO: Make this threshold a local param
        ball_too_far = (
            np.linalg.norm(world_state.ball.pos - self.robot.pose[0:2]) > 0.15
        )

        if ball_too_far or (
            dot_product > KICK_DOT_THRESHOLD
            and np.linalg.norm(world_state.ball.vel) > KICK_BALL_SPEED_THRESHOLD
        ):
            return True
        return False
