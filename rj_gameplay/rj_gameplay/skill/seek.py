import argparse
import py_trees
import sys
import time

import stp.skill as skill
import stp.role as role
import stp.action as action
from action_behavior import ActionBehavior
from seek_behavior import SeekBehavior
import stp.rc as rc
import numpy as np

class ISeek(skill.ISkill, ABC):
    ...

class Seek(ISeek):
	"""
	A seeker skill based off of some heuristic
	"""

    def __init__(self, role: role.Role, heuristic: Callable[np.array, float]) -> None:
        self.robot = role.robot
        root = py_trees.composites.Sequence("Sequence")
        seek_behavior = SeekBehavior(heuristic)
        root.add_children(seek_behavior)
        root.setup_with_descendants()

    def tick(self, world_state: rc.WorldState, robot: rc.Robot) -> None:
        root.tick_once(robot)
