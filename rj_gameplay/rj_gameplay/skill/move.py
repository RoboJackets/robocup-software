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
from stp.action_behavior import ActionBehavior
import stp.rc as rc

class IMove(skill.ISkill, ABC):
    ...


"""
A skill version of move so that actions don't have to be called in tactics
"""
class Move(IMove):
    
    def __init__(self, role: stp.Role, pos: np.array):
        self.robot = role.robot
        self.move = action.move.Move(pos)
        self.move_behavior = ActionBehavior('Move', self.move)
        self.root = self.move_behavior
        self.root.setup_with_descendants()

    def tick(self, world_state) -> None:
        self.root.tick_once(self.robot)
        # TODO: change so this properly returns the actions intent messages
