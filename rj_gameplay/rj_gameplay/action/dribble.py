"""This module contains the interface and action for dribble."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.action as action
import stp.rc as rc
from rj_msgs.msg import RobotIntent
from typing import Optional
from rj_msgs import msg

class Dribble(action.IAction): #add ABC if fails
    """
    Dribble action
    """
    def __init__(self,
            robot_id : int,
            target_point : np.ndarray,
            target_vel : np.ndarray = np.array([0.0,0.0]),
            face_angle : Optional[float] = None,
            face_point : Optional[np.ndarray] = None,
            priority : int = 0) -> None:

        self.robot_id = robot_id
        self.target_point = target_point
        self.target_vel = target_vel
        self.face_angle = face_angle
        self.face_point = face_point
        self.priority = priority


    def is_done(self, world_state: rc.WorldState) -> bool:
        threshold = 0.3
        if self.robot_id is None or world_state is None:
            return False
        elif(math.sqrt((world_state.our_robots[self.robot_id].pose[0] - self.target_point[0])**2 + (world_state.our_robots[self.robot_id].pose[1] - self.target_point[1])**2) < threshold):
            return True
        else:
            return False

    def tick(self, intent: msg.RobotIntent) -> msg.RobotIntent:
        path_command = PathTargetMotionCommand()
        path_command.target.position = Point(x=self.target_point[0],y=self.target_point[1])
        path_command.target.velocity = Point(x=self.target_vel[0],y=self.target_vel[1])
        if(self.face_angle is not None):
            path_command.override_angle=[self.face_angle]

        if(self.face_point is not None):
            path_command.override_face_point=[Point(x=self.face_point[0], y=self.face_point[1])]

        intent.motion_command.path_target_command = [path_command]

        intent.dribbler_speed = 1.0
        intent.is_active = True
        return intent
