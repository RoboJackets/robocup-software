import py_trees
import random
import random stp.action as action
from typing import Optional, Any
import numpy as np


"""
A behavior for skills with cost functions, for skills like seek or mark
"""
class CostBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, heuristic: Callable[np.array, float]) -> None:

        self.heuristic = heuristic
        super(CostBehavior, self).__init__(name)


    def initialise(self) -> None:
        pass

    def update(self) -> py.trees.common.Status:
        """
        Uses heursitc to optimizes and find best position to move to
        Creates new move action to get robot to move there
        Always returns running
        """
        return py_trees.common.Status.RUNNING 
        

    def terminate(self, new_status):
        pass