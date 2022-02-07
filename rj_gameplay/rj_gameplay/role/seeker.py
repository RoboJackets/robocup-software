import stp
import math
from operator import xor 
from enum import Enum
from rj_gameplay.skill import move, seek
from rj_gameplay.situation.decision_tree import analyzer
from rj_msgs.msg import RobotIntent
from stp.utils.constants import RobotConstants

class SeekerRole(stp.role.Role):
    def __init__(self, robot: stp.rc.Robot) -> None:
        super().__init__(robot)

        self.move_skill = None

        # TODO: make FSM class (or at least use enum instead of str literals)
        self._state = "init"

        self._target_point = None

    def possible_target_point(self, world_state: rc.WorldState) -> List:

        SAG_DIST = RobotConstants.RADIUS * 0.75
        pointList : list
        box_w = world_state.field.def_area_long_dist_m
        box_h = world_state.field.def_area_short_dist_m
        opp_box_yrange = world_state.floor_length_m - box_h #the y cooirdinate at which oppenent goalie box starts
        field_x_gap = (world_state.floor_width_m - box_w)/2 # gap in between goalie box and the field width
        box_xright = world_state.floor_width_m - field_x_gap #the right x cooirdinate at which goalie box starts
        box_xleft = field_x_gap #the left x cooirdinate at which goalie box starts

        #exclusive or logic gate to allow robot to have values in certain x or y ranges but can not have both x and y in those ranges
        check = operator.xor((point[0] in np.arrange(box_xleft, box_xright)) , (point[1] in np.arrange(opp_box_yrange,world_state.field.floor_length_m)))
        #target point is not in either range should also be available 
        neither = (point[0] not in np.arrange(box_xleft, box_xright)) and (point[1] not in np.arrange(opp_box_yrange,world_state.field.floor_length_m))

        for angle in range(0, 360): #num represents passing angle 
            point = SAG_DIST * np.array([np.cos(angle), np.sin(angle)])
            if check == True or neither == True:
                pointList.append(point)
        
        return pointList
    
    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:
        """
        Assume our_robot has ball on init. Then:
         - on init: get every point in the OFFENSE section of field away from their_robots at a certain distance and move there

        """

        opp_pos = world_state.their_robots[target_robot_id].pose[0:2]

        for point in pointList:
            distvec = opp_pos - point
            dist = np.linalg.norm(distvec)
            if (dist > SAG_DIST and world_state.our_robot.target_point != point):
                self.target_point = point
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



