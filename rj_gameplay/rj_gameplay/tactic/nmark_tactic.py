from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar

import stp

from rj_gameplay.role import marker, capture_role
import numpy as np

import stp.global_parameters as global_parameters

from rj_msgs.msg import RobotIntent


def get_opponents_to_mark(world_state: stp.rc.WorldState, num_markers: int):
    # TODO: need to be optimized
    ball_pt = world_state.ball.pos

    dist_to_opponents = {
        np.linalg.norm(ball_pt - robot.pose[0:2]): robot
        for robot in world_state.their_robots
    }
    return [
        dist_to_opponents[dist]
        for dist in sorted(dist_to_opponents.keys())[0:num_markers]
    ]


class NMarkTactic(stp.tactic.Tactic):
    def __init__(self, world_state: stp.rc.WorldState, num_markers: int):
        super().__init__(world_state)
        self.num_markers = num_markers

        self.opponents_to_mark = get_opponents_to_mark(world_state, self.num_markers)

        for i in range(len(self.opponents_to_mark)):
            if i == 0:
                self._role_requests.append(
                    (
                        stp.role.cost.PickClosestToPoint(
                            self.opponents_to_mark[i].pose[:2]
                        ),
                        capture_role.CaptureRole,
                    )
                )
            else:
                self._role_requests.append(
                    (
                        stp.role.cost.PickClosestToPoint(
                            self.opponents_to_mark[i].pose[:2]
                        ),
                        marker.MarkerRole,
                    )
                )

    def init_roles(self, world_state: stp.rc.WorldState) -> None:
        self.assigned_roles = []
        for i, robot in enumerate(self.assigned_robots):
            role = self._role_requests[i][1]
            if role is capture_role.CaptureRole:
                self.assigned_roles.append(role(robot))
            elif role is marker.MarkerRole:
                self.assigned_roles.append(role(robot, self.opponents_to_mark[i]))

    def tick(self, world_state: stp.rc.WorldState) -> List[Tuple[int, RobotIntent]]:

        self.opponents_to_mark = get_opponents_to_mark(world_state, self.num_markers)

        if len(self._role_requests) != len(self.assigned_roles):
            self.init_roles(world_state)

        robot_intents = []
        for i in range(len(self.assigned_roles)):
            role = self.assigned_roles[i]
            if role.robot is not None:
                if i == 0:
                    robot_intents.append((role.robot.id, role.tick(world_state)))
                else:
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
