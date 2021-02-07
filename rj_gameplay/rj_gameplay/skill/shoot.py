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

class IShoot(skill.ISkill, ABC):
    ...

"""
A shoot skill which aims at the goal and shoots
"""
class Shoot(IShoot):

    def __init__(self, role: role.Role) -> None:
        self.robot = role.robot
        self.root = py_trees.composites.Sequence("Sequence")
        self.capture = action.Capture()
        self.pivot = action.Pivot(robot.pos, rc.Field.their_goal_loc)
        self.kick = action.Kick(rc.Field.their_goal_loc)
        self.capture_behavior = ActionBehavior('Capture', self.capture)
        #Add more logic for aiming and kicking
        self.pivot_behavior = ActionBehavior('Pivot', self.pivot) 
        self.kick_behavior = ActionBehavior('Kick', self.kick)
        self.root.add_children([self.capture_behavior, self.pivot_behavior, self.kick_behavior])
        self.root.setup_with_descendants()

    def tick(self) -> None:
        self.root.tick_once()

    
