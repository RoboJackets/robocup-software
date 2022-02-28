import stp
from rj_gameplay.role import seeker
from typing import List, Tuple
import numpy as np

from rj_msgs.msg import RobotIntent


class BasicSeek(stp.tactic.Tactic):
    """Seeks to a single point, passed in on init."""

    def __init__(self, world_state: stp.rc.WorldState):
        super().__init__(world_state)

        goalpoint = world_state.field.their_goal_loc
        self._role_requests.append(
            (stp.role.cost.PickClosestToPoint(goalpoint), seeker.SeekerRole)
        )

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[Tuple[int, RobotIntent]]:
        # returns list of (robot_id, robot_intent)

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
        robot = self.assigned_robots[0]
        role = self._role_requests[0][1]
        if role is dumb_move.DumbMove:
            self.assigned_roles.append(role(robot, self._seek_pt, world_state.ball.pos))
