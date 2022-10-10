from typing import List

import stp
from rj_gameplay.skill import move
from rj_msgs.msg import RobotIntent
import numpy as np


class RunnerRole(stp.role.Role):
    def __init__(self, robot: stp.rc.Robot) -> None:
        super().__init__(robot)
        self.move_skill = None
        self.target_point = None
        # self.robot_id = robot.id

    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:
        # current_pos = world_state.our_robots[self.robot_id].pose
        if (self.move_skill is None or self.move_skill.is_done(world_state)):
            self.target_point = self.next_point(world_state)
        
            # assign move skill
            self.move_skill = move.Move(
                robot=self.robot,
                target_point=self.target_point,
                face_point=self.target_point
            )

        intent = self.move_skill.tick(world_state)

        return intent
        
        # world_state.field.our_left_corner
    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        return False
    
    def next_point(self, world_state: stp.rc.WorldState):
        if ((self.target_point == world_state.field.our_right_corner).all()):
            return world_state.field.our_left_corner
        if ((self.target_point == world_state.field.our_left_corner).all()):
            return world_state.field.their_left_corner
        if ((self.target_point == world_state.field.their_left_corner).all()):
            return world_state.field.their_right_corner
        
        return world_state.field.our_right_corner