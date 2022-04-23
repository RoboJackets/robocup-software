from typing import List, Tuple

import stp
from rj_msgs.msg import RobotIntent

from rj_gameplay.role import dumb_move


class DumbTactic(stp.tactic.Tactic):
    def __init__(self, world_state: stp.rc.WorldState, pts: list):
        super().__init__(world_state)
        self.move_points = pts
        # request closest robot every pt
        for pt in self.move_points:
            self._role_requests.append(
                (stp.role.cost.PickClosestToPoint(pt), dumb_move.DumbMove)
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
            pt = self.move_points[i]
            if role is dumb_move.DumbMove:
                self.assigned_roles.append(role(robot, pt, world_state.ball.pos))
