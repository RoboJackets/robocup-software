from typing import List, Tuple

import numpy as np
import stp
from rj_msgs.msg import RobotIntent

from rj_gameplay.role import seeker


class BasicSeek(stp.tactic.Tactic):
    """Seeks to a single point, passed in on init."""

    def __init__(
        self,
        world_state: stp.rc.WorldState,
        num_seekers: int,
        formations: List,
        centroids: List,
    ):
        super().__init__(world_state)

        formation = formations
        centroid_list = centroids
        self._used_regions = []
        self._used_centroids = []
        self._num_seekers = num_seekers

        for i in range(self._num_seekers):
            if i not in range(len(formation)): break
            my_region = formation[i]
            self._used_regions.append(my_region)
            centroid = centroid_list[i]
            self._used_centroids.append(centroid)
            self._role_requests.append(
                (stp.role.cost.PickClosestToPoint(centroid), seeker.SeekerRole)
            )

    def init_roles(
        self,
        world_state: stp.rc.WorldState,
    ):
        for i, robot in enumerate(self.assigned_robots):
            role = self._role_requests[i][1]  # TODO: make this an actual type
            my_region = self._used_regions[i]
            centroid = self._used_centroids[i]
            if role is seeker.SeekerRole:
                self.assigned_roles.append(role(robot, my_region, centroid))

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[Tuple[int, RobotIntent]]:
        # returns list of (robot_id, robot_intent)

        # assumes all roles requested are filled, because tactic is one unit
        if len(self.assigned_roles) != len(self._role_requests):
            self.init_roles(world_state)

        return [(role.robot.id, role.tick(world_state)) for role in self.assigned_roles]

    @property
    def needs_assign(self):
        return False

    def is_done(
        self,
        world_state: stp.rc.WorldState,
    ) -> bool:
        return all([role.is_done(world_state) for role in self.assigned_roles])
