import py_trees
import random
import stp.action as action
# from rj_gameplay.action import move as Move
# from rj_gameplay.skill import move
from rj_gameplay.action import move
import stp.rc as rc
from typing import Optional, Any, Callable
import numpy as np



class CostBehavior(py_trees.behaviour.Behaviour):
    """
    A behavior for skills which use the move action based on a heuristic function, for skills like seek or mark
    """
    def __init__(self, name: str, heuristic: Callable[[np.array], float], robot: rc.Robot=None, ctx=None) -> None:
        self.heuristic = heuristic
        self.robot = robot
        self.ctx = ctx
        super(CostBehavior, self).__init__(name)

    def tick_once(self, robot: rc.Robot, ctx=None):
        """
        Ticks its action using the robot given (if root) or the robot from its parent.
        """
        self.robot = robot
        self.ctx = ctx
        super().tick_once()

    def initialise(self) -> None:
        pass

    def update(self) -> py_trees.common.Status:
        """
        Uses heursitc to optimizes and find best position to move to
        Creates new move action to get robot to move there
        Always returns running
        """
        # self.move = move.Move(target_point = np.array([0,1+self.robot.id]))
        # self.move.tick(self.robot)
        self.move = move.Move(self.robot.id, target_point = np.array([0.0,1.0+self.robot.id]))
        self.move.tick(self.robot)
        return py_trees.common.Status.RUNNING 
        

    def terminate(self, new_status):
        pass
