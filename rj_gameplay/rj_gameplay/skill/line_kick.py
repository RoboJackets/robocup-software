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
        actions = self.root.tick_once(self.robot, world_state)
        return actions
        # TODO: change so this properly returns the actions intent messages

    def is_done(self, world_state: rc.WorldState):
        # skill is done after move + kick
        return self.line_kick_action.is_done(world_state)