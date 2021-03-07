from abc import ABC, abstractmethod
from typing import Callable

import rj_gameplay.eval as eval
import argparse
import py_trees
import sys
import time
import numpy as np

import stp.skill as skill
import stp.role as role
import stp.action as action
from stp.skill.action_behavior import ActionBehavior
from stp.skill.cost_behavior import CostBehavior
import stp.rc as rc

class IMark(skill.ISkill, ABC):
    ...

"""
A skill which marks a given opponent robot according to some heuristic cost function
"""
class Mark(IMark):

    def __init__(self, mark_heuristic: Callable[[np.array], float]):
        self.__name__ = 'Mark'
        self.mark_heuristic = mark_heuristic
        self.mark_behavior = CostBehavior('Mark', self.mark_heuristic)
        self.root = self.mark_behavior
        self.root.setup_with_descendants()

    def tick(self, world_state: rc.WorldState, robot: rc.Robot) -> None:
        print('marking')
        # Print for stub
        self.root.tick_once(robot)
