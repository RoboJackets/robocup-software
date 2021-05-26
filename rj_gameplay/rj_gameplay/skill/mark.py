from abc import ABC, abstractmethod
from typing import Callable, Optional

import rj_gameplay.eval as eval
import argparse
import py_trees
import sys
import time
import numpy as np

import stp.skill as skill
import stp.role as role
import stp.action as action
from rj_gameplay.action import move
from stp.skill.action_behavior import ActionBehavior
import stp.rc as rc
import stp.utils.constants as constants

from rj_geometry_msgs.msg import Point, Segment

def get_mark_point(world_state: rc.WorldState, mark_robot_id: int):
    # workaround for non-working CostBehavior: 
    # initialize move action, update target point every tick (target point being opponent robot pos)

    # TODO: use mark_heuristic & CostBehavior to handle marking
    # argument: mark_heuristic: Callable[[np.array], float]
    # > self.mark_heuristic = mark_heuristic

    # dist away from target_robot to mark
    # TODO: add to global param server
    SAG_DIST = constants.Robot.Radius * 0.5

    # find point between ball and target robot that leaves SAG_DIST between edges of robots
    ball_pos = world_state.ball.pos
    opp_pos = world_state.their_robots[mark_robot_id].pose[0:2]

    mark_dir = (ball_pos - opp_pos) / np.linalg.norm(ball_pos - opp_pos)
    mark_pt = opp_pos + mark_dir * (2.0 * constants.Robot.Radius + SAG_DIST)

    return mark_pt 

class IMark(skill.ISkill, ABC):
    ...

"""
A skill which marks a given opponent robot according to some heuristic cost function
"""
class Mark(IMark):

    def __init__(self,
            robot : rc.Robot = None,
            target_point : np.ndarray = np.array([0.0,0.0]),
            target_vel : np.ndarray = np.array([0.0,0.0]),
            face_angle : Optional[float] = None,
            face_point : Optional[np.ndarray] = None):

        self.__name__ = 'Mark Skill'
        self.robot = robot

        self.target_point = target_point
        if self.robot is not None:
            self.move = move.Move(self.robot.id, target_point, target_vel, face_angle, face_point)
        else:
            self.move = move.Move(self.robot, target_point, target_vel, face_angle, face_point)

        self.mark_behavior = ActionBehavior('Mark', self.move)
        self.root = self.mark_behavior
        self.root.setup_with_descendants()

    def tick(self, robot: rc.Robot, world_state: rc.WorldState) -> None:
        self.robot = robot

        # update target point every tick to match movement of ball & target robot
        if world_state and world_state.ball.visible:
            self.target_point = get_mark_point(world_state, 1)
            self.move.target_point = self.target_point

        actions = self.root.tick_once(robot, world_state)
        return actions

    def is_done(self, world_state):
        return self.move.is_done(world_state)
