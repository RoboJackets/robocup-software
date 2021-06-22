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
from stp.skill.rj_sequence import RjSequence as Sequence
import stp.rc as rc

def calc_move_point(ball_pos: np.array, target_point: np.array) -> np.array:
    target_to_ball = target_point - ball_pos
    target_to_ball_half = (3 * target_to_ball) / (4 * np.linalg.norm(target_to_ball))
    return ball_pos - target_to_ball_half

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
    def __init__(self, robot: rc.Robot, target_point: np.array) -> None:
        self.robot = robot

        self.target_point = target_point
        self.move_point = None
        if self.robot is not None:
            self.line_kick_action = line_kick.LineKickAction(self.robot.id, self.target_point)
            self.move = move.Move(self.robot.id, self.move_point)
        else:
            self.line_kick_action = line_kick.LineKickAction(None, self.target_point)
            self.move = move.Move(None, self.move_point)

        # put into a tree
        self.line_kick_action_behavior = ActionBehavior('LineKick', self.line_kick_action)
        self.move_action_behavior = ActionBehavior('Move Behind', self.move)
        self.root = Sequence("Sequence")
        self.root.add_children([self.move_action_behavior, self.line_kick_action_behavior])
        self.root.setup_with_descendants()

    def tick(self, robot: rc.Robot, world_state: rc.WorldState) -> None:

        self.robot = robot
        self.move.target_point = calc_move_point(world_state.ball.pos, self.target_point)
        actions = self.root.tick_once(self.robot, world_state)
        self.move.robot_id = self.robot.id
        print(self.move.robot_id)
        self.line_kick_action.robot_id = self.robot.id
        return actions
        # TODO: change so this properly returns the actions intent messages

    def is_done(self, world_state: rc.WorldState):
        # skill is done after move + kick
        return self.line_kick_action.is_done(world_state)
