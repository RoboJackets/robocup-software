import py_trees
import random
import stp.action as action
from typing import Optional, Any
import stp.rc as rc


class ActionBehavior(py_trees.behaviour.Behaviour):
    """
    A behaviour for behviour trees which ticks its action when ticked
    """
    def __init__(self, name: str, action: action.IAction, robot: rc.Robot=None, ctx=None) -> None:
        self.action = action
        self.ctx = ctx
        self.robot = robot
        super(ActionBehavior, self).__init__(name)

    def tick_once(self, robot: rc.Robot, ctx=None) -> None:
        """
        Ticks its action using the robot given (if root) or the robot from its parent.
        This will probably become tick() or spin() once action server is implemented
        TODO: Should return a list of robot intents
        """
        self.robot = robot
        if robot is None:
            self.robot = self.parent.robot
        self.ctx = ctx
        super().tick_once()

    def initialise(self) -> None:
        """
        Begin spinning the action
        """
        self.action.spin()

    def update(self) -> py_trees.common.Status:
        """
        Check action and return the current state of the aciton
        TODO: Needs to somehow use robot intents to check on status of action
        """
        if self.robot is None:
            self.robot = self.parent.robot
        self.action.tick(self.robot, self.ctx)
        if not self.action.done():
            return py_trees.common.Status.RUNNING
        elif not self.action.fail():
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        pass