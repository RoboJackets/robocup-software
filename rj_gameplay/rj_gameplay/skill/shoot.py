from abc import ABC, abstractmethod

import rj_gameplay.eval as eval
import argparse
import py_trees
import sys
import time

import stp.skill as skill
import stp.role as role
import stp.action as action
from rj_gameplay.action import pivot, kick
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
        self.ctx = None
        self.__name__ = 'Shoot'
        self.root = sequence.RJSequence()
        self.pivot = pivot.Pivot(self.robot.pose[0:2], rc.Field.their_goal_loc)
        self.kick = kick.Kick(rc.Field.their_goal_loc)
        self.pivot_behavior = ActionBehavior('Pivot', self.pivot, self.robot) 
        self.kick_behavior = ActionBehavior('Kick', self.kick, self.robot)
        self.root.add_children([self.capture_behavior, self.pivot_behavior, self.kick_behavior])
        self.root.setup_with_descendants()

    def tick(self, robot:rc.Robot, ctx) -> None:
        return self.root.tick_once(robot, ctx)
     

    #if Kick and Pivot is done, Shoot should be done.
    def is_done(self, world_state: rc.WorldState) -> bool:
    	if self.pivot.is_done(world_state) and self.kick.is_done(world_state): 
    		return True
    	else:
    		return False

    
