from abc import ABC, abstractmethod

import rj_gameplay.eval as eval
import argparse
import py_trees
import sys
import time

import stp.skill as skill
import stp.role as role
import rj_gameplay.action as action
from stp.skill.action_behavior import ActionBehavior
import stp.rc as rc
import numpy as np

class IPivotKick(skill.ISkill, ABC):
    ...

class PivotKick(IPivotKick):
    """
    A pivot kick skill
    """

    def __init__(self, role: role.Role, target_point: np.array) -> None:
        self.robot = role.robot
        self.root = py_trees.composites.Sequence("Sequence")
        self.capture = action.capture.Capture()
        self.pivot = action.pivot.Pivot(self.robot.pos, self.robot.pose, target_point)
        self.kick = action.kick.Kick(target_point)
        self.capture_behavior = ActionBehavior('Capture', self.capture)
        self.pivot_behavior = ActionBehavior('Pivot', self.pivot) 
        self.kick_behavior = ActionBehavior('Kick', self.kick)
        self.root.add_children([self.capture_behavior, self.pivot_behavior, self.kick_behavior])
        self.root.setup_with_descendants()

    def tick(self, world_state: rc.WorldState, robot: rc.Robot) -> None:
        self.root.tick_once(robot)
        # TODO: change so this properly returns the actions intent messages
