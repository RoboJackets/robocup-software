from abc import ABC, abstractmethod

import rj_gameplay.eval as eval
import argparse
import py_trees
import sys
import time
import numpy as np
from typing import Optional

import stp.skill as skill
import stp.role as role
import stp.action as action
from rj_gameplay.action import move
from stp.skill.action_behavior import ActionBehavior
import stp.rc as rc
from rj_msgs import msg

class IKick(skill.ISkill, ABC):
    ...


"""
A skill version of move so that actions don't have to be called in tactics
"""
class Kick(IKick):
    
    def __init__(self,
            robot : rc.Robot = None,
            # in PivotKick there's a role.Role object that stores the robot
            # should that be matched here too?
            # if so, replace above with:
            # role: role.Role = None,
            target_point : np.ndarray = np.array([0.0,0.0])):

        self.__name__ = 'kick skill'

        # set robot, target_point of kick
        self.robot = robot
        # self.robot = role.robot # see question above 
        self.point = point 

        # create Kick action
        self.kick = action.Kick(target_point)
        # TODO: implement kick action
        self.kick_behavior = ActionBehavior('Kick', kick)

        # setup py_tree
        self.root = self.kick_behavior
        self.root.setup_with_descendants()

    def tick(self, robot: rc.Robot, world_state): #returns dict of robot and actions
        self.robot = robot
        actions = self.root.tick_once(self.robot, world_state)
        return actions
        # TODO: change so this properly returns the actions intent messages

    def is_done(self, world_state):
        return self.kick.is_done()
        # kick action has no params (yet?)
        # return self.kick.is_done(world_state)
