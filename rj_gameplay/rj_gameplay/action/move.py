from stp.action import IFiniteAction
import stp.rc as rc
from rj_msgs.msg import RobotIntent, MotionCommand, EmptyMotionCommand, PathTargetMotionCommand, LinearMotionInstant
from rj_geometry_msgs.msg import Point
import numpy as np
from typing import Optional
import rclpy

class Move(IFiniteAction):
    """
    Basic move, used for moves avoiding all obstacles.
    """

    def __init__(self, publisher:  rclpy.publisher,
            robot_id : int,
            target_point : np.ndarray,
            target_vel : np.ndarray = np.array([0.0,0.0]),
            face_angle : Optional[float] = None,
            face_point : Optional[np.ndarray] = None,
            dribbler_speed : float = 0.0,
            local_obstacles : int = 0,
            priority : int = 0):

        #TODO, make local obstacles a proper thing

        self.robot_id = robot_id
        self.intent = RobotIntent()

        self.intent.motion_command.empty_command = []

        path_command = PathTargetMotionCommand()
        path_command.target.position = Point(x=target_point[0],y=target_point[1])
        path_command.target.velocity = Point(x=target_vel[0],y=target_vel[1])
        if(face_angle is not None):
            path_command.target.face_angle=[face_angle]

        if(face_point is not None):
            path_command.target.face_point=[Point(face_point[0], face_point[1])]
            
        self.intent.motion_command.path_target_command = [path_command]
        self.intent.motion_command.pivot_command = []
        self.intent.motion_command.settle_command = []
        self.intent.motion_command.collect_command = []
        self.intent.motion_command.line_kick_command = []
        self.intent.motion_command.intercept_command = []

        self.intent.dribbler_speed = dribbler_speed
        self.intent.is_active = True

    def tick(self) -> None:
        self.publisher.publish(self.intent)

    def is_done(self, world_state, threshold=0.08, time=0.4) -> bool:
        if()

        pass

    def finish(self) -> None:
        intent = RobotIntent()
        intent.motion_command.empty_motion_command = EmptyMotionCommand()
        self.publisher.publish(intent)

