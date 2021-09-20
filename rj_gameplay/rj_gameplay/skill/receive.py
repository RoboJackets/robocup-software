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
# import stp.action as action
from rj_gameplay.action import receive, capture
from rj_msgs.msg import RobotIntent, SettleMotionCommand
from stp.skill.action_behavior import ActionBehavior
from stp.skill.rj_sequence import RjSequence as Sequence
import stp.rc as rc
from rj_msgs import msg

# class IReceive(skill.ISkill, ABC):
    


"""
A skill version of receive so that actions don't have to be called in tactics
"""

class Receive(skill.Iskill):
    def __init__(self,
            robot:rc.Robot = None,
            robot_id: int = None):

        self.robot = robot
        self.robot_id = robot_id
        self.has_ball_ticks = 0
        
        if self.robot is not None:
            self.receive = receive.Receive(self.robot.id)
            self.capture = capture.Capture(self.robot.id)
        else:
            self.receive = receive.Receive()
            self.capture = capture.Capture()
        self.receive_behavior = ActionBehavior('Receive', self.receive)
        self.capture_behavior = ActionBehavior('Capture', self.capture)
        self.root = Sequence('Sequence')
        self.root.add_children([self.receive_behavior, self.capture_behavior])
        self.root.setup_with_descendants()
        self.__name__ = 'receive skill'

    def tick(self, robot:rc.Robot, world_state:rc.WorldState, intent): #returns dict of robot and actions
        settle_command = SettleMotionCommand()
        intent.motion_command.settle_command = [settle_command]
        intent.dribbler_speed = 1.0
        intent.is_active = True
        self.robot = robot
        actions = self.root.tick_once(self.robot, world_state)
        self.capture.robot_id = self.robot.id
        self.receive.robot_id = self.robot.id
        return actions

    def is_done(self, world_state:rc.WorldState):
        if self.robot_id is not None and world_state.our_robots[
                self.robot_id].has_ball_sense:
            self.ticks_done += 1
        else:
            self.ticks_done -= 5
        self.ticks_done = np.clip(self.ticks_done, a_min=0, a_max=200)
        return self.ticks_done > 50


    def __str__(self):
        return f"Receive(robot={self.robot.id if self.robot is not None else '??'}, ticks={self.capture.ticks_done})"



class Receive(skill.Iskill):
    def __init__(self,
            robot:rc.Robot = None):

        self.robot = robot
        if self.robot is not None:
            self.receive = receive.Receive(self.robot.id)
            self.capture = capture.Capture(self.robot.id)
        else:
            self.receive = receive.Receive()
            self.capture = capture.Capture()
        self.receive_behavior = ActionBehavior('Receive', self.receive)
        self.capture_behavior = ActionBehavior('Capture', self.capture)
        self.root = Sequence('Sequence')
        self.root.add_children([self.receive_behavior, self.capture_behavior])
        self.root.setup_with_descendants()
        self.__name__ = 'receive skill'

    def tick(self, robot:rc.Robot, world_state:rc.WorldState): #returns dict of robot and actions
        self.robot = robot
        actions = self.root.tick_once(self.robot, world_state)
        self.capture.robot_id = self.robot.id
        self.receive.robot_id = self.robot.id
        return actions

    def is_done(self, world_state:rc.WorldState):

        return self.capture.is_done(world_state)

    def __str__(self):
        return f"Receive(robot={self.robot.id if self.robot is not None else '??'}, ticks={self.capture.ticks_done})"
