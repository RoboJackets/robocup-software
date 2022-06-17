from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar

import numpy as np
import stp
import stp.global_parameters as global_parameters
import stp.rc as rc
import stp.skill as skill
from rj_msgs.msg import RobotIntent

# TODO: replace w/ global param server
from stp.utils.constants import BallConstants, RobotConstants

import rj_gameplay.eval
import rj_gameplay.skill as skills
from rj_gameplay.role import dumb_move
from rj_gameplay.skill import move


class WallTactic(stp.tactic.Tactic):
    def __init__(self, world_state: stp.rc.WorldState, num_wallers: int):
        super().__init__(world_state)

        self.num_wallers = num_wallers

        self.wall_pts = self.find_wall_pts(self.num_wallers, world_state)

        # request closest robot every pt
        for pt in self.wall_pts:
            self._role_requests.append(
                (stp.role.cost.PickClosestToPoint(pt), dumb_move.DumbMove)
            )

    def init_roles(self, world_state: stp.rc.WorldState) -> None:
        for i, robot in enumerate(self.assigned_robots):
            role = self._role_requests[i][1]
            pt = self.wall_pts[i]
            if role is dumb_move.DumbMove:
                self.assigned_roles.append(role(robot, pt, world_state.ball.pos))

    def find_wall_pts(
        self, num_wallers: int, world_state: rc.WorldState
    ) -> List[np.ndarray]:
        """Calculates num_wallers points to form a wall between the ball and goal.
        :return list of wall_pts (as numpy arrays)
        """
        # TODO: param server this const
        # TODO: param server any constant from stp/utils/constants.py (this includes BallConstants)
        ball_pt = world_state.ball.pos
        goal_pt = world_state.field.our_goal_loc

        WALL_SPACING = 2 * RobotConstants.RADIUS + BallConstants.RADIUS

        # dist is slightly greater than def_area box bounds
        box_w = world_state.field.def_area_long_dist_m
        box_h = world_state.field.def_area_short_dist_m
        line_w = world_state.field.line_width_m * 2
        MIN_WALL_RAD = RobotConstants.RADIUS * 4.0 + line_w + np.hypot(box_w / 2, box_h)

        # get vector to define walling direction
        ball_goal_dir = (ball_pt - goal_pt) / (np.linalg.norm(ball_pt - goal_pt) + 1e-9)
        wall_dir = np.array(
            [ball_goal_dir[1], -ball_goal_dir[0]]
        )  # perp of above vector

        # find endpoints of wall
        mid_pt = goal_pt + (ball_goal_dir * MIN_WALL_RAD)
        end_pts = [
            mid_pt - num_wallers // 2 * WALL_SPACING * wall_dir,
            mid_pt + num_wallers // 2 * WALL_SPACING * wall_dir,
        ]

        # assign points furthest from vert center of field first
        #
        # the first condition (regarding ball's x coord) ensures that the wall
        # dir doesn't toggle rapidly when ball is near x=0
        if (abs(ball_pt[0]) > 0.5) and abs(end_pts[1][0]) > abs(end_pts[0][0]):
            end_pts[0], end_pts[1] = end_pts[1], end_pts[0]

        # linearly interpolate between endpoints to fill wall
        return [pt for pt in np.linspace(end_pts[0], end_pts[1], num_wallers)]

    def tick(
        self, world_state: stp.rc.WorldState
    ) -> List[Tuple[int, RobotIntent]]:  # (id, intent)
        self.wall_pts = self.find_wall_pts(self.num_wallers, world_state)

        # assumes all roles requested are filled, because tactic is one unit
        if len(self.assigned_roles) != len(self._role_requests):
            self.init_roles(world_state)

        robot_intents = []
        # for i in range(len(self.assigned_roles)):
        for i in range(self.num_wallers):
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
