from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar

from rj_gameplay.role import dumb_move

import stp
import rj_gameplay.eval
import rj_gameplay.skill as skills
from rj_gameplay.skill import move
import stp.skill as skill
import numpy as np

# TODO: replace w/ global param server
from stp.utils.constants import RobotConstants, BallConstants
import stp.global_parameters as global_parameters

from rj_msgs.msg import RobotIntent

# TODO: move this out of calculations file and into this tactic
from rj_gameplay.calculations import wall_calculations

MIN_WALL_RAD = None


class WallTactic(stp.tactic.Tactic):
    def __init__(self, world_state: stp.rc.WorldState, num_wallers: int):
        super().__init__(world_state)

        self.num_wallers = num_wallers

        self.wall_pts = wall_calculations.find_wall_pts(self.num_wallers, world_state)

        # request closest robot every pt
        for pt in self.wall_pts:
            self._role_requests.append(
                (stp.role.cost.PickClosestRobot(pt), dumb_move.DumbMove)
            )

    def init_roles(self, world_state: stp.rc.WorldState) -> None:
        for i, robot in enumerate(self.assigned_robots):
            role = self._role_requests[i][1]
            pt = self.wall_pts[i]
            if role is dumb_move.DumbMove:
                self.assigned_roles.append(role(robot, pt, world_state.ball.pos))

    def tick(
        self, world_state: stp.rc.WorldState
    ) -> List[Tuple[int, RobotIntent]]:  # (id, intent)
        self.wall_pts = wall_calculations.find_wall_pts(self.num_wallers, world_state)

        # assumes all roles requested are filled, because tactic is one unit
        if len(self.assigned_roles) != len(self._role_requests):
            self.init_roles(world_state)

        robot_intents = []
        for i in range(len(self.assigned_roles)):
            role = self.assigned_roles[i]
            wall_pt = self.wall_pts[i]
            if role.robot is not None:
                robot_intents.append(
                    (role.robot.id, role.tick(world_state, target_point=wall_pt))
                )
        return robot_intents

    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        # wall never ends (until basic defense decides)
        return False
