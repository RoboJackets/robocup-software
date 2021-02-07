import py_trees
import random
import random stp.action as action
from typing import Optional, Any


class ActionBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, action: action.Action) -> None:
        """
        Takes in an action and returns a status telling if the action is done yet
        """
        self.action = action
        super(ActionBehavior, self).__init__(name)


    def initialise(self) -> None:
        """
        Being spinning the tactic
        """
        action.tick()

    def update(self) -> py.trees.common.Status:
        """
        Check action and return the current state of the aciton
        """
        if not action.done():
            return py_trees.common.Status.RUNNING
        elif not action.fail():
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        pass