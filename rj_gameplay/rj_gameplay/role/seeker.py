import stp
import numpy as np
from typing import List
from rj_gameplay.skill import move
from rj_msgs.msg import RobotIntent
from stp.utils.constants import RobotConstants

class fieldRegions:
    def __init__(self, world_state: stp.rc.WorldState, formation: List):
        self.formationList = formation
    
    def getRegion(self, regionNum):
        if(regionNum ==1):
            return self.formationList[0]
        elif(regionNum == 2):
            return self.formationList[1]
        elif(regionNum == 3):
            return self.formationList[1]
        elif(regionNum == 4):
            return self.formationList[1]
        elif(regionNum == 5):
            return self.formationList[1]
        else:
            return "Invalid region number"
    
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
        box_w = world_state.field.def_area_long_dist_m
        box_h = world_state.field.def_area_short_dist_m
        y_quarter = world_state.floor_length_m/4
        y_3quarter = world_state.floor_length_m - y_quarter
        field_y = world_state.floor_length_m
        box_xright = world_state.field.def_area_x_right_coord
        box_xleft = world_state.field.def_area_x_left_coord
        field_xleft = world_state.field.bot_left_field_loc[0]
        field_xright = world_state.field.bot_right_field_loc[0]
        center_xleft = world_state.field.center_field_loc[0] - world_state.field.center_diameter_m
        center_xright = world_state.field.center_field_loc[0] + world_state.field.center_diameter_m
        center_yup = world_state.field.center_field_loc[1] + world_state.field.center_diameter_m
        center_ydown = world_state.field.center_field_loc[0] - world_state.field.center_diameter_m
        center = world_state.field.center_field_loc

        """
        Hard Code the Region Limits 
            -starting with the top left region being the first element, top right, center, bottom left, and then bottom right in order
            -each region's points must follow the same pattern (top left, top right...)
        """
        Xformation = [
            #Region 1 
            [[field_xleft, field_y], [box_xleft, field_y], [field_xleft, y_3quarter], [box_xleft, y_3quarter]],
            #Region 2
            [[box_xright, field_y], [field_xright, field_y], [box_xright, y_3quarter], [field_xright, y_3quarter]],
            #Region 3
            [[center_xleft, center_yup], [center_xright, center_yup], [center_xleft, center_ydown], [center_xright, center_ydown]]
            #Region 4
            [[field_xleft, y_quarter], [box_xleft, y_quarter], [field_xleft, 0], [box_xleft, 0]],
            #Region 5
            [[box_xright, y_quarter], [field_xright, y_quarter], [box_xright, 0], [field_xright, 0]]
        ]

        seekRegion = fieldRegions(world_state, Xformation)

        bounds = seekRegion.getRegion(self.field_region)
        left_xbound = bounds[0][0]
        right_xbound = bounds[1][0]
        upper_ybound = bounds[0][1]
        lower_ybound = bounds[2][1]

        opp_in_region = []
        #check to see if any opp robot in the region called
        for opp in world_state.their_robots:
            opp_pos = world_state.their_robots[opp].pose[0:2]
            if(opp_pos[0] in range(left_xbound, right_xbound, RobotConstants.RADIUS) 
                and opp_pos[1] in range(lower_ybound, upper_ybound, RobotConstants.RADIUS)):
                opp_in_region.append(opp_pos)
            else:
                break
        #research/ask about algorithm that does this task without a large time complexity
        if (len(opp_in_region.length) == 0):
            point = np.array((left_xbound + right_xbound/2, upper_ybound + lower_ybound/2))
            self.target_point = point
        else:
            for x in range(left_xbound, right_xbound, RobotConstants.RADIUS):
                for y in range(lower_ybound, upper_ybound, RobotConstants.RADIUS):
                    point = np.array((x,y))
                    for pos in opp_in_region:
                        SAG_DIST = RobotConstants.RADIUS * 0.25
                        distvec = pos - point
                        dist = np.linalg.norm(distvec)
                        if (dist > SAG_DIST):
                            self.target_point = point
                        else:
                            break
                    break
                break
        
        self.move_skill = move.Move(
                robot=self.robot,
                target_point=self.target_point,
                face_point=world_state.ball.pos,
            )

        intent = self.move_skill.tick(world_state)

        return intent

    def is_done(self, world_state) -> bool:
        return self._state == "seek_done"



