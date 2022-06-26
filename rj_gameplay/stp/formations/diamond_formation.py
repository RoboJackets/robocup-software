from typing import List

import numpy as np

import stp
from stp.utils.constants import RobotConstants


class DiamondFormation(stp.formations.Formations):
    def __init__(self, world_state: stp.rc.WorldState):
        super().__init__(world_state)

    @property
    def get_regions(self) -> List:
        """
        Hard Coded Region Bounds
            - starting with the top left region being the first element, top right, center, bottom left, and then bottom right in order
            - bounds are x min, x max, y min, y max
        """
        diamond_formation = [
            # Region 1 Points (STRIKER)
            (
                self.goalpost_left,
                self.goalpost_right,
                self.y_3quarter,
                self.their_def_area_short_dist_m - 2 * RobotConstants.RADIUS,
            ),
            # Region 2 Points (LEFT WING)
            (
                self.field_xleft + 2 * RobotConstants.RADIUS,
                self.box_xleft,
                self.center_ydown,
                self.center_yup,
            ),
            # Region 3 Points (CENTER)
            (
                self.center_xleft,
                self.center_xright,
                self.center_ydown,
                self.center_yup,
            ),
            # Region 4 Points (RIGHT WING)
            (
                self.box_xright,
                self.field_xright - 2 * RobotConstants.RADIUS,
                self.center_ydown,
                self.center_yup,
            ),
            # Region 5 Points (SWEEPER)
            (
                self.box_xleft,
                self.box_xright,
                self.our_def_area_short_dist_m + 2 * RobotConstants.RADIUS,
                self.y_quarter,
            ),
        ]

        return diamond_formation

    @property
    def get_centroids(self) -> List:
        """
        Convenience function for getting center of each region in get_regions()
        :return: list of region centroids for DiamondFormation
        """
        centroids = []
        regions = self.get_regions
        for region in regions:
            centroid = np.array(
                [
                    ((region[0] + region[1]) / 2),
                    ((region[2] + region[3]) / 2),
                ]
            )
            centroids.append(centroid)

        return centroids
