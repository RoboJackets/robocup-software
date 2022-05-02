from typing import List, Tuple

import numpy as np
import stp
from rj_msgs.msg import RobotIntent

from rj_gameplay.role import dumb_move
import stp.utils.constants as const


from rj_gameplay.role import dumb_move


class LineTactic(stp.tactic.Tactic):
    """Tactic for line up play that puts chosen robots in a line from start point to end point selected.
    Assuming that the liners are more than 2"""

    def __init__(
        self,
        world_state: stp.rc.WorldState,
        num_liners: int,
        start: np.ndarray,
        end: np.ndarray,
    ):
        super().__init__(world_state)

        # compute move points
        self.start = start
        self.end = end
        gap = np.linalg.norm(self.end - self.start)
        if gap <= 2 * (num_liners - 1) * const.RobotConstants.RADIUS:
            raise ValueError(
                "Start and end point is not valid for the number of robots assigned."
            )

        xpts = np.linspace(self.start[0], self.end[0], num_liners)
        ypts = np.linspace(self.start[1], self.end[1], num_liners)
        self.target_points = []

        for x, y in zip(xpts, ypts):
            target = np.array([x, y])
            self.target_points.append(target)
            self._role_requests.append(
                (stp.role.cost.PickClosestToPoint(target), dumb_move.DumbMove)
            )

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[Tuple[int, RobotIntent]]:
        # returns list of (robot_id, robot_intent)

        # if not self.assigned_roles:
        # assumes all roles requested are filled, because tactic is one unit
        if len(self.assigned_roles) != len(self._role_requests):
            self.init_roles(world_state)

        # return [(role.robot.id, role.tick(world_state)) for role in self.assigned_roles]
        robot_intents = []
        for i in range(len(self.assigned_roles)):
            role = self.assigned_roles[i]
            if role.robot:
                robot_intents.append((role.robot.id, role.tick(world_state)))
        return robot_intents

    def is_done(
        self,
        world_state: stp.rc.WorldState,
    ) -> bool:
        return False

    def init_roles(
        self,
        world_state: stp.rc.WorldState,
    ):
        for i, robot in enumerate(self.assigned_robots):
            role = self._role_requests[i][1]
            pt = self.target_points[i]
            if role is dumb_move.DumbMove:
                self.assigned_roles.append(role(robot, pt, world_state.ball.pos))
