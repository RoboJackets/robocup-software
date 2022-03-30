import argparse
import sys
import time
from abc import ABC

import numpy as np
import stp.action as action
import stp.rc as rc
import stp.skill as skill
from stp.skill.action_behavior import ActionBehavior


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
        target_point: np.ndarray = np.array([0.0, 0.0]),
    ):
        self.robot = robot
        self.target_point = target_point

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

    def __str__(self):
        return f"Capture(robot={self.robot.id if self.robot is not None else '??'}, target={self.target_point})"

    def __repr__(self) -> str:
        return self.__str__()
