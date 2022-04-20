from typing import List, Tuple

import numpy as np
import stp
from rj_msgs.msg import RobotIntent

from rj_gameplay.role import dumb_move


class PrepMove(stp.tactic.Tactic):
    """Seeks to a single point, passed in on init."""

    def __init__(self, world_state: stp.rc.WorldState):
        super().__init__(world_state)

        self._target_pt = np.array([0.0, 7.0])

        self._role_requests.append(
            (stp.role.cost.PickClosestToPoint(self._target_pt), dumb_move.DumbMove)
        )

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[Tuple[int, RobotIntent]]:
        # returns list of (robot_id, robot_intent)

        # assumes all roles requested are filled, because tactic is one unit
        self._target_pt = world_state.ball.pos[0:2] - [0, 0.5]
        if len(self.assigned_roles) != len(self._role_requests) or self.assigned_robots:
            self.init_roles(world_state)

        return [(role.robot.id, role.tick(world_state)) for role in self.assigned_roles]

    def is_done(
        self,
        world_state: stp.rc.WorldState,
    ) -> bool:
        if self.assigned_roles is not None:
            return all([role.is_done(world_state) for role in self.assigned_roles])
        return False

    def init_roles(
        self,
        world_state: stp.rc.WorldState,
    ):
        robot = self.assigned_robots[0]
        role = self._role_requests[0][1]
        if role is dumb_move.DumbMove:
            self.assigned_roles.append(
                role(robot, self._target_pt, world_state.ball.pos)
            )
