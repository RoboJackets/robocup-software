from typing import List, Tuple

import numpy as np
import stp
from stp.utils.constants import RobotConstants


class Formations:
    def __init__(self, world_state: stp.rc.WorldState):
        self.y_quarter: float = world_state.field.length_m / 4
        self.y_3quarter: float = world_state.field.length_m - self.y_quarter
        self.field_y: float = world_state.field.length_m
        self.box_xright: float = world_state.field.def_area_x_right_coord
        self.box_xleft: float = world_state.field.def_area_x_left_coord
        self.field_xleft: float = world_state.field.bot_left_field_loc[0]
        self.field_xright: float = world_state.field.bot_right_field_loc[0]
        self.center_xleft: float = (
            world_state.field.center_field_loc[0] - world_state.field.center_diameter_m
        )
        self.center_xright: float = (
            world_state.field.center_field_loc[0] + world_state.field.center_diameter_m
        )
        self.center_yup: float = (
            world_state.field.center_field_loc[1] + world_state.field.center_diameter_m
        )
        self.center_ydown: float = (
            world_state.field.center_field_loc[1] - world_state.field.center_diameter_m
        )

    @property
    def get_x_formation(self) -> List:
        """
        Hard Code the Region Bounds
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

    def get_centroid(self, region: List) -> np.ndarray:
        """
        Conveniance function for getting center of a region
        :return: center of the region
        """
        centroid = np.array(
            [
                ((region[0] + region[1]) / 2),
                ((region[2] + region[3]) / 2),
            ]
        )
        return centroid
