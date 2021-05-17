from abc import ABC, abstractmethod

import rj_gameplay.eval as eval
import argparse
import py_trees
import sys
import time

import stp.skill as skill
import stp.role as role
import stp.action as action
from rj_gameplay.action import move
from rj_gameplay.action import kick 
from stp.skill.action_behavior import ActionBehavior
import stp.rc as rc

class ILineKick(skill.ISkill, ABC):
    ...

class LineKick(ILineKick):
    """
    Move through the ball and shoot.
    """

    # role-based implementation
    # def __init__(self, role: role.Role) -> None:
        # self.robot = role.robot
    # role-blind implementation
    def __init__(self, robot: rc.Robot, world_state: rc.WorldState) -> None:
        self.robot = robot

        r_id = None
        if robot:
            r_id = self.robot.id
        # move to ball
        self.move = move.Move(r_id, world_state.ball.pos)
        self.move_behavior = ActionBehavior('Move', self.move)
        # kick once there
        self.kick = kick.Kick(r_id)
        self.kick_behavior = ActionBehavior('Kick', self.kick)

        # setup sequence of move -> kick
        self.root = py_trees.composites.Sequence("Sequence")
        self.root.add_children([self.move_behavior, self.kick_behavior])
        self.root.setup_with_descendants()
        self.__name__ = "line kick skill"

    def tick(self, robot: rc.Robot, world_state: rc.WorldState) -> None:
        self.robot = robot
        actions = self.root.tick_once(self.robot, world_state)
        return actions
        # TODO: change so this properly returns the actions intent messages

    def is_done(self, world_state: rc.WorldState):
        # skill is done after move + kick
        return self.kick.is_done(world_state)
