from abc import ABC, abstractmethod
from typing import List

import stp


class Formations(ABC):
    def __init__(self, world_state: stp.rc.WorldState):
        self.y_quarter: float = world_state.field.length_m / 4
        self.field_y: float = world_state.field.length_m
        self.y_3quarter: float = self.field_y - self.y_quarter
        self.box_xright: float = world_state.field.def_area_x_right_coord
        self.box_xleft: float = world_state.field.def_area_x_left_coord
        self.field_xleft: float = world_state.field.our_left_corner[0]
        self.field_xright: float = world_state.field.our_right_corner[0]
        self.goalpost_left: float = world_state.field.our_goal_post_coordinates[0][0]
        self.goalpost_right: float = world_state.field.our_goal_post_coordinates[1][0]
        self.their_def_area_short_dist_m: float = (
            self.field_y - world_state.field.def_area_short_dist_m
        )
        self.our_def_area_short_dist_m: float = world_state.field.def_area_short_dist_m
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
    @abstractmethod
    def get_centroids(self) -> List:
        "Get the centroid associated with the called region"

    @property
    @abstractmethod
    def get_regions(self) -> List:
        """
        Get all of the region bounds of the formation. Each region is in order of left to right from top to bottom.
        Within each region the bounds are order as follows:
            x min,
            x max,
            y min,
            y max,
        """
