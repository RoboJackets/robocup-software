from typing import List, Tuple

import stp

from rj_gameplay.role import ball_move
import numpy as np

import stp.global_parameters as global_parameters

from rj_msgs.msg import RobotIntent


class BallMoveTactic(stp.tactic.Tactic):
    """
    Captures ball and shoots. This Tactic merely holds the StrikerRole and handles its assignment for the Play level.
    """

    def __init__(self, world_state: stp.rc.WorldState):
        super().__init__(world_state)

        self._role_requests.append(
            (
                stp.role.cost.PickClosestToPoint(world_state.ball.pos),
                ball_move.BallMoveRole,
            )
        )

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[Tuple[int, RobotIntent]]:
        # returns list of (robot_id, robot_intent)

        # assumes all roles requested are filled, because tactic is one unit
        if len(self.assigned_roles) != len(self._role_requests):
            self.init_roles(world_state)

        # if low performance, make this not a for loop since it's only one tactic
        return [(role.robot.id, role.tick(world_state)) for role in self.assigned_roles]

    def is_done(
        self,
        world_state: stp.rc.WorldState,
    ) -> bool:
        return all([role.is_done(world_state) for role in self.assigned_roles])

    def init_roles(
        self,
        world_state: stp.rc.WorldState,
    ):
        robot = self.assigned_robots[0]
        role = self._role_requests[0][1]
        if role is ball_move.BallMoveRole:
            self.assigned_roles.append(role(robot))

    @property
    def needs_assign(self):
        # never needs assign after init
        # TODO: make this + pass tac part of the superclass
        return False