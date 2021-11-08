from abc import ABC, abstractmethod

import rj_gameplay.eval as eval
import argparse
import py_trees
import sys
import time
import numpy as np

import stp.skill as skill
import stp.role as role
import stp.action as action
from rj_geometry_msgs.msg import Point
from rj_msgs.msg import RobotIntent, LineKickMotionCommand
import stp.rc as rc
from rj_gameplay.MAX_KICK_SPEED import *

class ILineKickSkill(skill.ISkill, ABC):
    ...


class LineKickSkill(ILineKickSkill):
    """
    A skill version of line kick so that actions don't have to be called in tactics
    """

    # role-based implementation
    # def __init__(self, role: role.Role) -> None:
    # self.robot = role.robot
    # role-blind implementation
    def __init__(self,
                 action_client_dict: Dict[Type[Any], List[Any]],
                 robot: rc.Robot,
                 target_point: np.array,
                 priority: int = 0,
                 chip: bool = False,
                 kick_speed: float = 5.5) -> None:
        self.robot = robot
        self.move_action_clients = self.action_client_dict.get(MoveActionClient)

        self.target_point = target_point
        self.priority = priority
        self.chip = chip
        self.kick_speed = kick_speed
        # self.kick_speed = 5.5




    def tick(self, robot: rc.Robot, world_state: rc.WorldState, intent: RobotIntent):
        self.robot = robot

        ball_to_target = self.target_point - world_state.ball.pos
        ball_to_target /= np.linalg.norm(ball_to_target)
        robot_dir = np.array([np.cos(robot.pose[2]), np.sin(robot.pose[2])])

        right_direction = np.dot(ball_to_target, robot_dir) > 0.9
        if not right_direction:
            self.kick_speed = 0.0
            
        line_kick_command = LineKickMotionCommand()
        line_kick_command.target = Point(x=self.target_point[0], y=self.target_point[1])
        intent.shoot_mode = RobotIntent.SHOOT_MODE_KICK if not self.chip else RobotIntent.SHOOT_MODE_CHIP
        intent.trigger_mode = RobotIntent.TRIGGER_MODE_ON_BREAK_BEAM
        if self.kick_speed <= MAX_KICK_SPEED:
            intent.kick_speed = self.kick_speed
        else:
            intent.kick_speed = MAX_KICK_SPEED

        intent.motion_command.line_kick_command = [line_kick_command]
        intent.is_active = True

        return {self.robot.id : intent}        
        # TODO: change so this properly returns the actions intent messages

    def is_done(self, world_state: rc.WorldState):
        # skill is done after move + kick
        return np.linalg.norm(world_state.ball.vel) > 1.0

    def __str__(self):
        return f"LineKick(robot={self.robot.id if self.robot is not None else '??'}, target={self.target_point}, chip={self.chip}, kick_speed={self.kick_speed})"
