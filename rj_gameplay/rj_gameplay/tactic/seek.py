from typing import List, Tuple

import stp
from rj_msgs.msg import RobotIntent

from rj_gameplay.role import seeker


class Seek(stp.tactic.Tactic):
    """Seeks to a single point, passed in on init."""

    def __init__(
        self,
        world_state: stp.rc.WorldState,
        num_seekers: int,
        formation: stp.formations.Formations,
    ):
        super().__init__(world_state)

        regions = formation.get_regions
        centroid_list = formation.get_centroids
        self._used_regions = []
        self._used_centroids = []
        self._num_seekers = num_seekers
        # TODO: make seeker be able to handle more robots than formation regions available OR make formations have more regions
        """
        This section of the code is clunky due to the constraint of formations. Currently, if there are more robots seeking than formation regions, it will assign multiple robots to regions by resetting the available regions.

        At the time of this role creation, we are in Divison B, which is why this is not a big issue. In the future, this must be handled according to the todo above.
        """
        reset = 0
        for i in range(self._num_seekers):
            if i not in range(len(regions)):
                i = reset
                reset += 1 if reset < 4 else 0
            my_region = regions[i]
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
