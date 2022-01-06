import stp.play
import stp.tactic

from rj_gameplay.tactic import move_tactic
import stp.skill
import stp.role
from stp.role.assignment.naive import NaiveRoleAssignment
import stp.rc
from typing import (
    Dict,
    List,
    Tuple,
    Optional,
    Type,
)
import numpy as np
from rj_msgs.msg import RobotIntent

# TODO: move this cost fn to its own module
class PickRobotById(stp.role.CostFn):
    def __init__(self, robot_id: int):
        self._robot_id = robot_id

    def __call__(
        self,
        robot: stp.rc.Robot,
        world_state: stp.rc.WorldState,
    ) -> float:

        if robot.id == self._robot_id:
            return 0.0

        # TODO: use max int or float('inf')
        return 1e9

    # TODO: rm this from stp/role/__init__.py
    def unassigned_cost_fn(
        self, prev_results: Optional["RoleResult"], world_state: stp.rc.WorldState
    ) -> float:
        pass

    def __repr__(self):
        return f"PickRobotById(robot={self._robot_id})"

class LineUp(stp.play.Play):
    """Play that lines up all six robots on the side of the field.
    """

    def __init__(self):
        print("init LineUp Play")
        super().__init__()

        # fill cost functions by priority
        # (this is a contrived example)
        self.ordered_costs = [PickRobotById(i) for i in range(6)]

        # fill roles
        self.ordered_roles = [move_tactic.Move for _ in range(6)]

        # filled by assign_roles() later
        self.ordered_tactics = [] 

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:

        # if no tactics created, assign roles and create them
        if not self.ordered_tactics:
            print(self.ordered_costs)
            print(self.ordered_roles)
            self.assign_roles(world_state)

        # return robot intents from assigned tactics back to gameplay node
        return self.get_robot_intents(world_state)

    def init_tactics(self, assigned_robots: List[stp.rc.Robot]) -> None:
        # TODO: consider moving this logic to outside this method, put in superclass, pass through kwargs

        # compute move points
        start = (3.0, 1.0)
        dy = 0.5
        move_points = [(start[0], start[1] + i*dy) for i in range(6)]

        for role, robot, pt in zip(self.ordered_roles, assigned_robots, move_points):
            kwargs = {'target_point': pt, 'face_point': (0.0, 0.0)}
            new_tactic = role(robot, **kwargs)
            self.ordered_tactics.append(new_tactic)


