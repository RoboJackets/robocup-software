from typing import List

import numpy as np

import stp
from stp.utils.constants import RobotConstants


class XFormation(stp.formations.Formations):
    def __init__(self, world_state: stp.rc.WorldState):
        super().__init__(world_state)

    @property
    def get_regions(self) -> List:
        """
        Hard Coded Region Bounds
            - starting with the top left region being the first element, top right, center, bottom left, and then bottom right in order
            - bounds are x min, x max, y min, y max
        """
        X_formation = [
            # Region 1 points (TOP LEFT)
            (
                self.field_xleft + 2 * RobotConstants.RADIUS,
                self.box_xleft - 2 * RobotConstants.RADIUS,
                self.y_3quarter,
                self.field_y - 2 * RobotConstants.RADIUS,
            ),
            # Region 2 points (TOP RIGHT)
            (
                self.box_xright + 2 * RobotConstants.RADIUS,
                self.field_xright - 2 * RobotConstants.RADIUS,
                self.y_3quarter,
                self.field_y - 2 * RobotConstants.RADIUS,
            ),
            # Region 3 points (CENTER CIRCLE)
            (
                self.center_xleft,
                self.center_xright,
                self.center_ydown,
                self.center_yup,
            ),
            # Region 4 points (BOTTOM LEFT)
            (
                self.field_xleft + 2 * RobotConstants.RADIUS,
                self.box_xleft - 2 * RobotConstants.RADIUS,
                0 + 2 * RobotConstants.RADIUS,
                self.y_quarter,
            ),
            # Region 5 points (BOTTOM RIGHT)
            (
                self.box_xright + 2 * RobotConstants.RADIUS,
                self.field_xright - 2 * RobotConstants.RADIUS,
                0 + 2 * RobotConstants.RADIUS,
                self.y_quarter,
            ),
        ]

        return X_formation

    @property
    def get_centroids(self) -> List:
        """
        Convenience function for getting center of each region in get_regions()
        :return: list of region centroids for XFormation
        """
        centroids = []
        regions = self.get_regions
        for region in regions:
            centroid = np.array([
                ((region[0] + region[1]) / 2),
                ((region[2] + region[3]) / 2),
            ])
            centroids.append(centroid)

        return centroids
