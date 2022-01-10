import py_trees
import random
import stp.action as action
from typing import Optional, Any
import stp.rc as rc


class RJSequence(py_trees.composites.Sequence):
    def __init__(self, robot: rc.Robot = None, ctx=None):
        self.robot = robot
        self.ctx = ctx
        super().__init__()

    def tick_once(self, robot: rc.Robot, ctx=None):
        self.robot = robot
        self.ctx = ctx
        super().tick_once()
