from typing import List, Tuple

import stp

from rj_gameplay.role import marker, capture_role
import numpy as np

import stp.global_parameters as global_parameters

from rj_msgs.msg import RobotIntent
import stp.rc
from typing import Set


def get_closest_unmarked_opp(pt: np.ndarray, already_marked: Set[stp.rc.Robot]):
    # if this is slow, optimize it (scipy or vectorize with numpy)
    min_dist_opps = sorted([
        np.linalg.norm(pt - robot.pose[0:2]): robot
        for robot in world_state.their_robots
    ])

    for robot in min_dist_opps:
        if robot not in already_marked:
            return robot

    print("no unmarked robots")
    return None


class NMarkTactic(stp.tactic.Tactic):
    def __init__(self, world_state: stp.rc.WorldState, num_markers: int):
        super().__init__(world_state)
        self.num_markers = num_markers

        already_marked = set()
        self.face_block_pts = []
        for i in range(num_markers):
            goal_pt = world_state.field.our_goal_loc
            opp_to_mark = get_closest_unmarked_opp(goal_pt, already_marked)

            # TODO: fix up capture so it feels confident in stealing ball from opp robots
            face_point = {"robot": opp_to_mark.id}
            block_point = {"goal": None}

            self._role_requests.append(
                (
                    stp.role.cost.PickClosestToPoint(
                        opp_to_mark.pose[:2]
                    ),
                    marker.MarkerRole,
                )
            )

            self.face_block_pts.append((face_point, block_point))

            already_marked.add(opp_to_mark)

    def init_roles(self, world_state: stp.rc.WorldState) -> None:
        self.assigned_roles = []
        for i, robot in enumerate(self.assigned_robots):
            role = self._role_requests[i][1]
            face_point, block_point = self.face_block_pts[i]
            if role is marker.MarkerRole:
                new_marker = role(face_point, block_point)
                self.assigned_roles.append(new_marker)

    def tick(self, world_state: stp.rc.WorldState) -> List[Tuple[int, RobotIntent]]:

        if len(self._role_requests) != len(self.assigned_roles):
            self.init_roles(world_state)

        robot_intents = []
        for i in range(len(self.assigned_roles)):
            role = self.assigned_roles[i]
            if role.robot is not None:
                robot_intents.append(
                    (
                        role.robot.id,
                        role.tick(
                            world_state, target_robot=self.opponents_to_mark[i]
                        ),
                    )
                )

        return robot_intents

    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        # While on the defense play, it always returns False
        return False
