from abc import ABC, abstractmethod

import rj_gameplay.eval as eval
import argparse
import py_trees
import sys
import time
import math
import numpy as np
from typing import Optional, Dict, Type, List, Any

import stp.skill
import stp.role as role
import stp.action as action
from rj_geometry_msgs.msg import Point
from rj_msgs.msg import RobotIntent, PathTargetMotionCommand
import stp.rc as rc

<<<<<<< HEAD
from rj_gameplay.action.move_action_client import MoveActionClient

"""
A skill version of move so that actions don't have to be called in tactics
"""
=======
>>>>>>> bce13ce53ddb2ecb9696266d980722c34617dc15

class Move(stp.skill.Skill):
    def __init__(
        self,
        action_client_dict: Dict[Type[Any], List[Any]],
        robot: Optional[rc.Robot] = None,
        target_point: np.ndarray = np.array([0.0, 0.0]),
        target_vel: np.ndarray = np.array([0.0, 0.0]),
        face_angle: float = None,
        face_point: np.ndarray = None,
        ignore_ball: bool = False,
        priority: int = 0,
    ):

        self.move_action_clients = action_client_dict.get(MoveActionClient)
        self.robot = robot
        self.target_point = target_point
        self.target_vel = target_vel
        self.face_point = face_point
        self.face_angle = face_angle
        self.ignore_ball = ignore_ball
        self.priority = priority

        self.cached_intent = None

        self.__name__ = "Move"

<<<<<<< HEAD
    def tick(self, robot: rc.Robot, world_state: rc.WorldState, intent: RobotIntent):
        self.robot = robot

=======
    def tick(self, world_state: rc.WorldState) -> RobotIntent:
        super().tick(world_state)
        intent = RobotIntent()
>>>>>>> bce13ce53ddb2ecb9696266d980722c34617dc15
        path_command = PathTargetMotionCommand()
        path_command.target.position = Point(
            x=self.target_point[0], y=self.target_point[1]
        )
        path_command.target.velocity = Point(x=self.target_vel[0], y=self.target_vel[1])
        path_command.ignore_ball = self.ignore_ball

        if self.face_angle is not None:
            path_command.override_angle = [self.face_angle]

        if self.face_point is not None:
            path_command.override_face_point = [
                Point(x=self.face_point[0], y=self.face_point[1])
            ]

        intent.motion_command.path_target_command = [path_command]
        intent.is_active = True
<<<<<<< HEAD

        return {self.robot.id: intent}
=======
>>>>>>> bce13ce53ddb2ecb9696266d980722c34617dc15

        # TODO: motion planning is a lot more stable when not being spammed with repeat intents, use Action Client/Server to avoid re-requests when the intent is the same
        return intent

    def is_done(self, world_state: rc.WorldState) -> bool:
        threshold = 0.3
        if self.robot:
            if self.robot.id is None or world_state is None:
                return False
            elif (
                math.sqrt(
                    (
                        world_state.our_robots[self.robot.id].pose[0]
                        - self.target_point[0]
                    )
                    ** 2
                    + (
                        world_state.our_robots[self.robot.id].pose[1]
                        - self.target_point[1]
                    )
                    ** 2
                )
                < threshold
            ):
                return True
            else:
                return False
        else:
            return True

    def __str__(self):
        ignore_ball_str = ", ignoring ball" if self.ignore_ball else ""
        return f"Move(robot={self.robot.id if self.robot is not None else '??'}, target={self.target_point}{ignore_ball_str})"

    def __repr__(self) -> str:
        return self.__str__()
