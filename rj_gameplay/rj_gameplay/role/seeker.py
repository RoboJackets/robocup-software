import stp
import numpy as np
from typing import List
from rj_gameplay.skill import move
from rj_msgs.msg import RobotIntent
from stp.utils.constants import RobotConstants


class fieldRegions:
    def __init__(self, formation: List):
        self.formationList = formation

    def getRegion(self, regionNum):
        if regionNum == 1:
            return self.formationList[0]
        elif regionNum == 2:
            return self.formationList[1]
        elif regionNum == 3:
            return self.formationList[1]
        elif regionNum == 4:
            return self.formationList[1]
        elif regionNum == 5:
            return self.formationList[1]
        else:
            return "Invalid region number"


def get_opp_inRegion(region: List, world_state: stp.rc.WorldState) -> List:
    opp_in_region = []
    # check to see if any opp robot in the region called
    for opp in world_state.their_robots:
        opp_pos = world_state.their_robots[opp].pose[0:2]
        if opp_pos[0] in range(
            region[0][0], region[0][1], RobotConstants.RADIUS
        ) and opp_pos[1] in range(region[1][0], region[1][1], RobotConstants.RADIUS):
            opp_in_region.append(opp_pos)
        else:
            break

    return opp_in_region


def get_open_point(region: List, opp_in_region: List) -> np.ndarray:
    # research/ask about algorithm that does this task without a large time complexity
    for x in range(region[0][0], region[0][1], RobotConstants.RADIUS):
        for y in range(region[1][0], region[1][1], RobotConstants.RADIUS):
            point = np.array((x, y))
            for pos in opp_in_region:
                SAG_DIST = RobotConstants.RADIUS * 0.5
                distvec = pos - point
                dist = np.linalg.norm(distvec)
                if dist > SAG_DIST:
                    bestpoint = point
                else:
                    break
            break
        break

    return bestpoint


class SeekerRole(stp.role.Role):
    def __init__(self, robot: stp.rc.Robot, regionNum: int) -> None:
        super().__init__(robot)
        self.move_skill = None
        self.field_region = regionNum
        # TODO: make FSM class (or at least use enum instead of str literals)
        self._state = "init"
        self._target_point = None

    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:
        """
        Assume our_robot has ball on init. Then:
         - on init: get every point in the OFFENSE section of field away from their_robots at a certain distance and move there

        """

        # TODO: make Formations class (with local variables below and X formation)
        y_quarter = world_state.floor_length_m / 4
        y_3quarter = world_state.floor_length_m - y_quarter
        field_y = world_state.floor_length_m
        box_xright = world_state.field.def_area_x_right_coord
        box_xleft = world_state.field.def_area_x_left_coord
        field_xleft = world_state.field.bot_left_field_loc[0]
        field_xright = world_state.field.bot_right_field_loc[0]
        center_xleft = (
            world_state.field.center_field_loc[0] - world_state.field.center_diameter_m
        )
        center_xright = (
            world_state.field.center_field_loc[0] + world_state.field.center_diameter_m
        )
        center_yup = (
            world_state.field.center_field_loc[1] + world_state.field.center_diameter_m
        )
        center_ydown = (
            world_state.field.center_field_loc[0] - world_state.field.center_diameter_m
        )

        """
        Hard Code the Region Bounds
            -starting with the top left region being the first element, top right, center, bottom left, and then bottom right in order
            -each region's bounds are set up as x and y bounds in a list
        """
        Xformation = [
            # Region 1 bounds
            [[field_xleft, box_xleft], [field_y, y_3quarter]],
            # Region 2 bounds
            [[box_xright, field_xright], [field_y, y_3quarter]],
            # Region 3 bounds
            [[center_xleft, center_xright], [center_yup, center_ydown]],
            # Region 4 bounds
            [[field_xleft, box_xleft], [y_quarter, 0]],
            # Region 5 bounds
            [[box_xright, field_xright], [y_quarter, 0]],
        ]

        seekRegion = fieldRegions(Xformation)

        bounds = seekRegion.getRegion(self.field_region)

        opp_in_region = get_opp_inRegion(bounds, world_state)

        if len(opp_in_region.length) == 0:
            point = np.array(
                (bounds[0][0] + bounds[0][1] / 2, bounds[1][0] + bounds[1][1] / 2)
            )
            self.target_point = point
        else:
            self.target_point = get_open_point(bounds, opp_in_region)

        self.move_skill = move.Move(
            robot=self.robot,
            target_point=self.target_point,
            face_point=world_state.ball.pos,
        )

        intent = self.move_skill.tick(world_state)

        return intent

    def is_done(self, world_state) -> bool:
        return self._state == "seek_done"
