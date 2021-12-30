import py_trees
import random
import stp.action as action
from typing import Optional, Any
import stp.rc as rc
from typing import TypedDict, List


class RobotActions(TypedDict):
    robot_id: int
    actions: List[action.IAction]


class ActionBehavior(py_trees.behaviour.Behaviour):
    """
    A behaviour for behviour trees which ticks its action when ticked
    """

    def __init__(
        self,
        name: str,
        action: action.IAction,
        robot: rc.Robot = None,
        ctx=None,
    ) -> None:
        self.action = action
        self.ctx = ctx
        self.robot = robot
        self.world_state = None
        super(ActionBehavior, self).__init__(name)

    def tick_once(
        self, robot: rc.Robot, world_state: rc.WorldState, ctx=None
    ) -> RobotActions:
        """
        Ticks its action using the robot given (if root) or the robot from its parent.
        This will probably become tick() or spin() once action server is implemented
        TODO: Should return a list of robot intents
        """
        if robot is not None:
            self.robot = robot
        else:
            self.robot = self.parent.robot
        self.action.robot_id = robot.id
        if world_state is not None:
            self.world_state = world_state
        else:
            self.world_state = self.parent.world_state
        # if robot is None:
        #     self.robot = self.parent.robot
        self.ctx = ctx
        super().tick_once()
        return {self.robot.id: [self.action]}

    def initialise(self) -> None:
        """
        Begin spinning the action
        TODO: Implement with action server
        """
        pass

    def update(self) -> py_trees.common.Status:
        """
        Check action and return the current state of the aciton
        TODO: Needs to somehow use robot intents to check on status of action
        """
        if not self.world_state is None and self.action.is_done(
            self.world_state
        ):
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        pass
