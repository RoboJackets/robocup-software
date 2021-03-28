from stp.action import IFiniteAction
import stp.rc as rc
from rj_msgs.msg import RobotIntent, MotionCommand, EmptyMotionCommand, PathTargetMotionCommand, LinearMotionInstant
from rj_geometry_msgs.msg import Point
import numpy as np
from typing import Optional
import rclpy
import time

class Pivot(IFiniteAction):
    """
    Basic Pivot, used for pivoting in place.
    """

    def __init__(self, publisher:  rclpy.publisher,
            robot_id : int,
            face_angle : Optional[float] = None,
            #face_point : Optional[np.ndarray] = None,
            dribbler_speed : float = 0.0,
            priority : int = 0):


        self.robot_id = robot_id
        self.intent = RobotIntent()

        self.intent.motion_command.empty_command = []
        self.intent.motion_command.path_target_command = []
        pivot_command = PivotCommand()

        if(face_angle is not None):
            pivot_command.target.face_angle=[face_angle]

        #if(face_point is not None):
        #    pivot_command.target.face_point=[Point(face_point[0], face_point[1])]

        self.intent.motion_command.pivot_command = [pivot_command]
        self.intent.motion_command.settle_command = []
        self.intent.motion_command.collect_command = []
        self.intent.motion_command.line_kick_command = []
        self.intent.motion_command.intercept_command = []

        self.intent.dribbler_speed = dribbler_speed
        self.intent.is_active = True

    def tick(self) -> None:
        self.publisher.publish(self.intent)

    def is_done(self, world_state)  -> bool:
        threshold=math.pi/12
        time=0.4
        if(world_state.our_robots[self.robot_id].pose[2] - self.face_angle < threshold):
            return True
        else:
            return False

        #TODO filter this using time

    def finish(self) -> None:
        intent = RobotIntent()
        intent.motion_command.empty_motion_command = EmptyMotionCommand()
        self.publisher.publish(intent)

