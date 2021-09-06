import argparse
import py_trees
import sys
import time
from abc import ABC, abstractmethod

import stp.skill as skill
import stp.role as role
import stp.action as action
import stp.rc as rc
import numpy as np
from typing import Callable

class Seek(skill.ISkill):
    """
    A seeker skill based off of some heuristic
    """
    def __init__(self, role: role.Role, heuristic: Callable[np.array, float]) -> None:
        # TODO: Properly implement using move action
        pass

    def tick(self, world_state: rc.WorldState, robot: rc.Robot) -> None:
        # TODO: change so this properly returns the actions intent messages
        pass

    def __str__(self):
        return f"Seek(robot={self.robot.id if self.robot is not None else '??'})"
