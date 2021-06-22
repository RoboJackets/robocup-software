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
from rj_gameplay.action import move, line_kick
from stp.skill.action_behavior import ActionBehavior
import stp.rc as rc

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
    def __init__(self, robot: rc.Robot, target_point: np.array, chip: bool = False, kick_speed: float = 6.0) -> None:
        self.robot = robot

        self.target_point = target_point
        self.chip = chip
        self.kick_speed = kick_speed

        if self.robot is not None:
            self.line_kick_action = line_kick.LineKickAction(self.robot.id, self.target_point, chip=chip, kick_speed=kick_speed)
        else:
            self.line_kick_action = line_kick.LineKickAction(None, self.target_point, chip=chip, kick_speed=kick_speed)

        # put into a tree
        self.line_kick_action_behavior = ActionBehavior('LineKick', self.line_kick_action)
        self.root = self.line_kick_action_behavior
        self.root.setup_with_descendants()

    def tick(self, robot: rc.Robot, world_state: rc.WorldState) -> None:
        self.robot = robot

        self.line_kick_action.target = self.target_point

        ball_to_target = self.target_point - world_state.ball.pos
        ball_to_target /= np.linalg.norm(ball_to_target)
        robot_dir = np.array([np.cos(robot.pose[2]), np.sin(robot.pose[2])])

        right_direction = np.dot(ball_to_target, robot_dir) > 0.9
        self.line_kick_action.kick_speed = self.kick_speed if right_direction else 0.0

        actions = self.root.tick_once(self.robot, world_state)
        return actions
        # TODO: change so this properly returns the actions intent messages

    def is_done(self, world_state: rc.WorldState):
        # skill is done after move + kick
        return self.line_kick_action.is_done(world_state)

    def __str__(self):
        return f"LineKick(robot={self.robot.id if self.robot is not None else '??'}, target={self.target_point}, chip={self.chip})"
