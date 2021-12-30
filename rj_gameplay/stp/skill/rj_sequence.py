import py_trees
import random
import stp.action as action
from typing import Optional, Any
import stp.rc as rc
from typing import List
from stp.skill.action_behavior import ActionBehavior, RobotActions


class RjSequence(py_trees.composites.Sequence):
    """
    A sequence block for our use
    This is currently needed since we need a way to send wolrdstate down the tree for actions's is_done() functions
    This will not be needed once the action server is implemented
    """

    def __init__(self, name: str) -> None:
        self.robot = None
        self.world_state = None
        self.curr_index = -1
        self.current_child = None
        super().__init__(name)

    def tick_once(
        self, robot: rc.Robot, world_state: rc.WorldState
    ) -> RobotActions:
        self.robot = robot
        self.world_state = world_state
        if self.current_child is None:
            self.current_child = self.children[0]
            self.curr_index = 0  # start sequence
        elif self.current_child.action.is_done(world_state):
            if self.curr_index < len(self.children):
                self.current_child = self.children[self.curr_index]
                self.curr_index += 1
            else:
                # restart sequence
                self.current_child = self.children[0]
                self.curr_index = 0  # start sequence

        actions = self.current_child.tick_once(robot, world_state)
        return actions
