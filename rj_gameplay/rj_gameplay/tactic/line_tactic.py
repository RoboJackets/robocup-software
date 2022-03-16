from typing import List, Tuple

import numpy as np
import stp
from rj_msgs.msg import RobotIntent

from rj_gameplay.role import dumb_move


class LineTactic(stp.tactic.Tactic):
    """Tactic for line up play that puts all six robots in a line on the left of the field.
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
        xpts = np.linspace(self.start[0], self.end[0], num_liners)
        ypts = np.linspace(self.start[1], self.end[1], num_liners)
        self.target_points = []

        for x, y in zip(xpts, ypts):
            target = np.array([x, y])
            self.target_points.append(target)
            self._role_requests.append(
                (stp.role.cost.PickClosestToPoint(target), dumb_move.DumbMove)
            )

        print(self.target_points)

        # TODO: make the # here a param instead of hardcoding for same reason as above TODO
        # self.move_points = [(start[0], start[1] + i * dy) for i in range(6)]

        # # request closest robot every pt
        # for pt in self.move_points:
        #     # for some reason stp.role doesn't need to be imported?
        #     self._role_requests.append(
        #         (stp.role.cost.PickClosestToPoint(pt), dumb_move.DumbMove)
        #     )

        # OR hardcode certain ids to go
        # for i, pt in enumerate(self.move_points):
        #     self._role_requests.append((stp.role.cost.PickRobotById(5-i), dumb_move.DumbMove))

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[Tuple[int, RobotIntent]]:
        # returns list of (robot_id, robot_intent)

        # if not self.assigned_roles:
        # assumes all roles requested are filled, because tactic is one unit
        if len(self.assigned_roles) != len(self._role_requests):
            self.init_roles(world_state)

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
        for i, robot in enumerate(self.assigned_robots):
            role = self._role_requests[i][1]
            pt = self.target_points[i]
            if role is dumb_move.DumbMove:
                self.assigned_roles.append(role(robot, pt, world_state.ball.pos))
