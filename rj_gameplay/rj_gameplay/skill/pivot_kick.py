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

    def __init__(self, robot: rc.Robot=None, target_point: np.array) -> None:
        self.robot = robot
        self.root = py_trees.composites.Sequence("Sequence")
        if robot is not None:
            self.pivot = action.Pivot(robot.pose[0:2], target_point)
        else:
            self.pivot = action.Pivot(np.array([0.0,0.0]), target_point)
        self.kick = action.Kick(target_point)
        self.pivot_behavior = ActionBehavior('Pivot', pivot) 
        self.kick_behavior = ActionBehavior('Kick', kick)
        self.root.add_children([pivot_behavior, kick_behavior])
        self.root.setup_with_descendants()

    def tick(self, robot: rc.Robot, world_state: rc.WorldState) -> None:
        self.robot = robot
        if robot is not None:
            self.pivot.pivot_point = robot.pose[0:2]
        actions = self.root.tick_once(robot)
        return actions
        # TODO: change so this properly returns the actions intent messages

    def is_done(self, world_state: rc.WorldState) -> bool:
        return self.kick.is_done(world_state)