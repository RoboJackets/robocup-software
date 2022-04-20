from typing import List, Tuple
from enum import Enum, auto

import numpy as np
import stp
from rj_msgs.msg import RobotIntent

from rj_gameplay.role import seeker


class State(Enum):
    SEEKING = auto()
    DONE = auto()

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
        self._state = State.SEEKING
        self._timer_started = False
        self._ticks_left = 60


        for i in range(self._num_seekers):
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

    def start_timer(self):
        self._timer_started = True

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[Tuple[int, RobotIntent]]:
        # returns list of (robot_id, robot_intent)

        role_intents = []

        if self._state == State.SEEKING:
            # assumes all roles requested are filled, because tactic is one unit
            if len(self.assigned_roles) != len(self._role_requests):
                self.init_roles(world_state)

            if self._timer_started:
                self._ticks_left -= 1

            if self._ticks_left <= 0 or all([role.is_done(world_state) for role in self.assigned_roles]):
                self._state = State.DONE

            role_intents = [(role.robot.id, role.tick(world_state)) for role in self.assigned_roles]

        return role_intents

    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        return self._state == State.DONE