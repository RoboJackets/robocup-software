from abc import ABC, abstractmethod

import rj_gameplay.eval as eval
import argparse
import py_trees
import sys
import time
from typing import Tuple

import stp.skill as skill
import stp.role as role
import stp.action as action
from rj_gameplay.action import intercept
from stp.skill.action_behavior import ActionBehavior
import stp.rc as rc
import numpy as np


class IIntercept(skill.ISkill, ABC):
    ...


"""
A skill version of intercept so that actions don't have to be called in tactics
"""


# TODO: discuss collapsing skills/actions
class Intercept(IIntercept):
    def __init__(
        self,
        robot: rc.Robot = None,
        target_point: Tuple = (0.0, 0.0),
    ):
        self.robot = robot
        self.target_point = np.asarray_chkfinite(target_point)

        self.__name__ = "Intercept Skill"
        if self.robot is not None:
            self.intercept = intercept.Intercept(self.robot.id, target_point)
        else:
            self.intercept = intercept.Intercept(None, target_point)
        self.intercept_behavior = ActionBehavior(
            "Intercept", self.intercept, self.robot
        )
        self.root = self.intercept_behavior
        self.root.setup_with_descendants()

    def tick(self, robot: rc.Robot, world_state: rc.WorldState):
        self.robot = robot
        self.intercept.robot_id = self.robot.id
        self.intercept.target_point = self.target_point
        return self.root.tick_once(self.robot, world_state)

    def is_done(self, world_state) -> bool:
        return self.intercept.is_done(world_state)
