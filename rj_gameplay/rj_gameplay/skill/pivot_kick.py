from abc import ABC, abstractmethod

import rj_gameplay.eval as eval
import argparse
import py_trees
import sys
import time

import stp.skill as skill
import stp.role as role
import stp.action as action
from stp.action_behavior import ActionBehavior
import stp.rc as rc

class IPivotKick(skill.ISkill, ABC):
    ...

class PivotKick(IPivotKick):
    """
    A pivot kick skill
    """

    def __init__(self, role: role.Role, target_point: np.array) -> None:
        self.robot = role.robot
        self.root = py_trees.composites.Sequence("Sequence")
        self.capture = action.Capture()
        self.pivot = action.Pivot(robot.pos, target_point)
        self.kick = action.Kick(target_point)
        self.capture_behavior = ActionBehavior('Capture', capture)
        self.pivot_behavior = ActionBehavior('Pivot', pivot) 
        self.kick_behavior = ActionBehavior('Kick', kick)
        self.root.add_children([capture_behavior, pivot_behavior, kick_behavior])
        self.root.setup_with_descendants()

    def tick(self, world_state: rc.WorldState, robot: rc.Robot) -> None:
        self.root.tick_once(robot)
