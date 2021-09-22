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
from stp.utils.constants import RobotConstants

from rj_geometry_msgs.msg import Point, Segment

def get_mark_point(target_robot_id: int, world_state: rc.WorldState):
    # workaround for non-working CostBehavior:
    # initialize move action, update target point every tick (target point being opponent robot pos)

    # TODO: use mark_heuristic & CostBehavior to handle marking
    # argument: mark_heuristic: Callable[[np.array], float]
    # > self.mark_heuristic = mark_heuristic

    # dist away from target_robot to mark
    # TODO: add to global param server
    SAG_DIST = RobotConstants.RADIUS * 0.5

    # find point between ball and target robot that leaves SAG_DIST between edges of robots
    # this will be mark point
    ball_pos = world_state.ball.pos
    opp_pos = world_state.their_robots[target_robot_id].pose[0:2]

    mark_dir = (ball_pos - opp_pos) / np.linalg.norm(ball_pos - opp_pos)
    mark_pos = opp_pos + mark_dir * (2 * RobotConstants.RADIUS + SAG_DIST)

    # if ball inside robot radius of mark_pos, can't mark normally
    if np.linalg.norm(mark_pos - ball_pos) < RobotConstants.RADIUS:
        # instead, get in front of opp robot holding ball
        mark_pos += mark_dir * 2 * RobotConstants.RADIUS

    return mark_pos

"""
A skill which marks a given opponent robot according to some heuristic cost function
"""
class Mark(skill.ISkill): #add ABC if fails

    def __init__(self, robot: rc.Robot = None, target_robot: rc.Robot = None) -> None:

        self.__name__ = 'Mark Skill'
        self.robot = robot
        self.target_robot = target_robot

        if self.robot is not None:
            self.move = move.Move(self.robot.id, np.array([0.0, 0.0]), np.array([0.0, 0.0]), None, None)
        else:
            self.move = move.Move(None, np.array([0.0, 0.0]), np.array([0.0, 0.0]), None, None)

        self.mark_behavior = ActionBehavior('Mark', self.move)
        self.root = self.mark_behavior
        self.root.setup_with_descendants()

    def tick(self, robot: rc.Robot, world_state: rc.WorldState) -> None:
        self.robot = robot

        # update target point every tick to match movement of ball & target robot
        if world_state and world_state.ball.visible:
            if self.target_robot is None:
                mark_point = get_mark_point(1, world_state)
            else:
                mark_point = get_mark_point(self.target_robot.id, world_state)

            if mark_point is None:
                return []
            self.move.target_point = mark_point
            self.move.face_point = world_state.ball.pos

        actions = self.root.tick_once(robot, world_state)
        return actions

    def is_done(self, world_state):
        return self.move.is_done(world_state)

    def __str__(self):
        return f"Mark(robot={self.robot.id if self.robot is not None else '??'}, target={self.target_robot.id if self.target_robot is not None else '??'})"
