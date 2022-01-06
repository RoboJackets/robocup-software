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
    """Play that lines up all six robots on the side of the field."""

    def __init__(self):
        super().__init__()

        # fill cost functions by priority
        # (this is a contrived example)
        self.ordered_costs = [PickRobotById(i) for i in range(6)]

        # fill roles
        self.ordered_roles = [move_tactic.MoveTactic for _ in range(6)]

        # filled by assign_roles() later
        self.ordered_tactics = []

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:

        # if no tactics created, assign roles and create them
        if not self.ordered_tactics:
            self.assign_roles(world_state)

        # return robot intents from assigned tactics back to gameplay node
        return self.get_robot_intents(world_state)

    def init_new_tactics(self, assigned_robots: List[stp.rc.Robot], world_state: stp.rc.WorldState) -> None:
        # compute move points
        start = (3.0, 0.0)
        dy = 0.5
        move_points = [(start[0], start[1] + i * dy) for i in range(6)]

        for role, robot, pt in zip(self.ordered_roles, assigned_robots, move_points):
            new_tactic = None
            # TODO: this is bad, shouldn't have to check types of tactics imo
            # the only role in this play is the move tactic, but in other plays there will be more
            # although perhaps this is fine since the play has to manually fill ordered_roles dynamically anyhow, indicating it knows what roles to expect
            if role is move_tactic.MoveTactic:
                new_tactic = role(robot, pt, (0.0, 0.0))

            if new_tactic is not None:
                self.ordered_tactics.append(new_tactic)
