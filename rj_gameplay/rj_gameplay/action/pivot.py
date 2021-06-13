"""This module contains the interface and action for pivot."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.action as action
import numpy as np
import stp.rc as rc
from rj_msgs.msg import RobotIntent, PivotMotionCommand
from rj_geometry_msgs.msg import Point



class Pivot(action.IFiniteAction):

    def __init__(self, robot_id:int, pivot_point:np.ndarray, target_point:np.ndarray, dribble_speed:float=1.0, priority:int=1):
        self.robot_id = robot_id
        self.pivot_point = pivot_point
        self.target_point = target_point
        self.dribble_speed = dribble_speed

    def tick(self, intent: RobotIntent) -> None:
        new_intent = intent
        pivot_command = PivotMotionCommand()
        pivot_command.pivot_point = Point(x=self.pivot_point[0], y=self.pivot_point[1])
        pivot_command.pivot_target = Point(x=self.target_point[0], y=self.target_point[1])
        new_intent.motion_command.pivot_command = [pivot_command]
        new_intent.dribbler_speed = self.dribble_speed
        new_intent.is_active = True
        return new_intent

    def is_done(self, world_state:rc.WorldState) -> bool:
        #TODO: Change this when we get action state feedback
        angle_threshold = 0.65
        stopped_threshold = 1*10**(-5)
        if self.robot_id is None:
            return False
        robot = world_state.our_robots[self.robot_id]
        robot_pos = robot.pose[0:2]
        robot_pos_unit =  robot_pos / np.linalg.norm(robot_pos)
        target_point_unit = self.target_point / np.linalg.norm(self.target_point)
        dot_product = np.dot(robot_pos_unit, target_point_unit)
        angle = np.arccos(dot_product)
        if abs(angle - angle_threshold) < angle_threshold and abs(world_state.our_robots[self.robot_id].twist[2]) < stopped_threshold:
            return True
        else:
            return False
