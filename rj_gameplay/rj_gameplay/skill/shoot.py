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
from stp.skill.action_behavior import ActionBehavior, RobotActions
from stp.skill.rj_sequence import RjSequence as Sequence
import stp.rc as rc
import numpy as np

class IShoot(skill.ISkill, ABC):
    ...

"""
A shoot skill which aims at the goal and shoots
"""
random_shoot = random.uniform(-0.5,0.5)

class Shoot(IShoot):

    def __init__(self, chip: bool, kick_speed: float, target_point: np.array = np.array([random_shoot, 12.])) -> None:
        self.robot: rc.Robot = None
        self.ctx = None
        self.target_point = target_point
        self.__name__ = 'Shoot'
        self.root = Sequence("Sequence")
        if self.robot is not None:
            self.pivot = pivot.Pivot(self.robot.id, self.robot.pose[0:2], target_point, 1.0)
            self.kick = kick.Kick(self.robot.id, chip, kick_speed)
        else:
            self.pivot = pivot.Pivot(self.robot, np.array([0., 0.]), target_point, 1.0)
            self.kick = kick.Kick(self.robot, chip, kick_speed)
        self.pivot_behavior = ActionBehavior('Pivot', self.pivot, self.robot) 
        self.kick_behavior = ActionBehavior('Kick', self.kick, self.robot)
        self.root.add_children([self.pivot_behavior, self.kick_behavior])
        self.root.setup_with_descendants()

    def tick(self, robot: rc.Robot, world_state: rc.WorldState) -> RobotActions:
        self.robot = robot
        self.pivot.pivot_point = world_state.ball.pos
        return self.root.tick_once(robot, world_state)
     

    #if Kick and Pivot is done, Shoot should be done.
    def is_done(self, world_state: rc.WorldState) -> bool:
    	if self.pivot.is_done(world_state) and self.kick.is_done(world_state): 
    		return True
    	else:
    		return False

    
