"""Holds functions used by plays that use the wall tactic."""

from dataclasses import dataclass
from typing import List, Optional
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar

import stp.action as action
import stp.rc as rc
import stp.tactic as tactic
import stp.role as role

import rj_gameplay.eval
import rj_gameplay.skill as skills
from rj_gameplay.skill import move
import stp.skill as skill
import numpy as np
# TODO: replace w/ global param server
from stp.utils.constants import RobotConstants, BallConstants
import stp.global_parameters as global_parameters

MIN_WALL_RAD = None


def find_wall_pts(num_wallers: int,
                  world_state: rc.WorldState) -> List[np.ndarray]:
    global MIN_WALL_RAD
    """Calculates num_wallers points to form a wall between the ball and goal.
    :return list of wall_pts (as numpy arrays)
    """
    # TODO: param server this const
    # TODO: param server any constant from stp/utils/constants.py (this includes BallConstants)
    ball_pt = world_state.ball.pos
    goal_pt = world_state.field.our_goal_loc

    WALL_SPACING = BallConstants.RADIUS

    # dist is slightly greater than def_area box bounds
    box_w = world_state.field.def_area_long_dist_m
    box_h = world_state.field.def_area_short_dist_m
    line_w = world_state.field.line_width_m * 2
    MIN_WALL_RAD = RobotConstants.RADIUS + line_w + np.hypot(box_w / 2, box_h)

    # get direction vec
    dir_vec = (ball_pt - goal_pt) / np.linalg.norm(ball_pt - goal_pt)
    wall_vec = np.array([dir_vec[1], -dir_vec[0]])

    # find mid_pt
    mid_pt = goal_pt + (dir_vec * MIN_WALL_RAD)
    wall_pts = [mid_pt]

    # set wall points in middle out pattern, given wall dir vector and WALL_SPACING constant
    wall_pts = [mid_pt]
    for i in range(num_wallers - 1):
        mult = i // 2 + 1
        delta = (mult * (2 * RobotConstants.RADIUS + WALL_SPACING)) * wall_vec
        if i % 2: delta = -delta
        wall_pts.append(mid_pt + delta)

    return wall_pts
