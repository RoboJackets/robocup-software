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
from rj_gameplay.skill import settle, capture, intercept
from rj_msgs.msg import RobotIntent, SettleMotionCommand
import stp.rc as rc
from rj_msgs import msg

class Receive(skill.ISkill):
    def __init__(self, robot: rc.Robot = None):
        self.robot = robot
        self.__name__ = 'receive skill'

        # receive = intercept -> capture
        self.intercept = intercept.Intercept(robot)
        self.capture = capture.Capture(robot)

        # TODO: enum or type or smth here besides int
        #       this is an FSM, basically
        self.curr_step = 0

    def tick(self, robot: rc.Robot, world_state: rc.WorldState,
             intent: RobotIntent):

        if robot is not None:
            print(robot.id)
            print(self.curr_step)

        self.intercept.robot = robot
        self.capture.robot = robot

        # TODO: use behavior tree construct for this lol
        if self.curr_step == 0:
            print(self.intercept.is_done(world_state))
            if self.intercept.is_done(world_state):
                self.curr_step = 1
                print("int done switch to cap")
                return self.capture.tick(robot, world_state, intent)
            return self.intercept.tick(robot, world_state, intent)
        elif self.curr_step == 1:
            if self.capture.is_done(world_state):
                self.curr_step = 2
                print("cap done")
            return self.capture.tick(robot, world_state, intent)

    def is_done(self, world_state:rc.WorldState) -> bool:
        if self.curr_step >= 2:
            self.curr_step = 0
            return True
        return False


    def __str__(self):
        return f"Receive[Intercept(robot={self.intercept.robot.id if self.intercept.robot is not None else '??'}), Capture(robot={self.capture.robot.id if self.capture.robot is not None else '??'})]"
