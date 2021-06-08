from abc import ABC, abstractmethod

import rj_gameplay.eval as eval
import argparse
import py_trees
import sys
import time

import stp.skill as skill
import stp.role as role
import stp.action as action
from rj_gameplay.action import capture, pivot, kick
from stp.skill.action_behavior import ActionBehavior
import stp.rc as rc
import stp.skill.sequence as sequence

class IShoot(skill.ISkill, ABC):
    ...

"""
A shoot skill which aims at the goal and shoots
"""
class Shoot(IShoot):

    def __init__(self) -> None:
        self.robot: rc.Robot = None
        self.__name__ = 'Shoot'
        self.root = sequence.RJSequence()
        self.capture = capture.Capture()
        self.pivot = pivot.Pivot(self.robot.pose[0:2],[1,1], rc.Field.their_goal_loc)
        self.kick = kick.Kick(rc.Field.their_goal_loc)
        self.capture_behavior = ActionBehavior('Capture', self.capture ,self.robot)
        #Add more logic for aiming and kicking
        self.pivot_behavior = ActionBehavior('Pivot', self.pivot, self.robot) 
        self.kick_behavior = ActionBehavior('Kick', self.kick, self.robot)
        self.root.add_children([self.capture_behavior, self.pivot_behavior, self.kick_behavior])
        self.root.setup_with_descendants()

    def tick(self, world_state: rc.WorldState, robot:rc.Robot) -> None:
        self.root.tick_once(robot)
        # TODO: change so this properly returns the actions intent messages

    
