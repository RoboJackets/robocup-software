"""Contains the stub for the move tactic. """

from dataclasses import dataclass
from typing import List, Optional
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar

import stp.action as action
import stp.rc as rc
import stp.tactic as tactic
import stp.role as role

import rj_gameplay.eval
import rj_gameplay.skill as skills
from rj_gameplay.skill import move
import stp.skill as skill
import numpy as np

from rj_msgs.msg import RobotIntent

MAX_ROBOT_VELOCITY = 3.0


class move_cost(role.CostFn):
    """
    A cost function for how to choose a striker
    TODO: Implement a better cost function
    """

    def __init__(self, target_point: np.ndarray, cost_scale: float = 1.0):
        self.target_point = target_point
        self.cost_scale = cost_scale

    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:
        return (
            (
                (robot.pose[0] - self.target_point[0]) ** 2
                + (robot.pose[1] - self.target_point[1]) ** 2
            )
            / MAX_ROBOT_VELOCITY
            * self.cost_scale
        )

    def unassigned_cost_fn(
        self,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        return role.BIG_STUPID_NUMBER_CONST_FOR_UNASSIGNED_COST_PLS_CHANGE


class MoveTactic(tactic.Tactic):
    def __init__(self, robot: rc.Robot, target_point, face_point):
        super().__init__(robot)

        self.target_point = target_point
        self.face_point = face_point

        self.move_skill = None

    def tick(self, world_state: rc.WorldState) -> RobotIntent:
        # create skill with correct target & face_point
        if self.move_skill is None:
            self.move_skill = move.Move(
                robot=self.robot,
                target_point=self.target_point,
                face_point=self.face_point,
            )

        # tick skill and return
        intent = self.move_skill.tick(world_state)
        return intent

    def is_done(self, world_state):
        return self.move_skill.is_done(world_state)
