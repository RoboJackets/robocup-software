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
    ratio = 0.9 # pct of line robot should be on (1.0 = on mark_pt)

    ball_pos = world_state.ball.pos

    print("-"*81)
    print(ball_pos)
    # print(world_state.their_robots)
    # print(world_state.our_robots)
    # if len(world_state.their_robots) is 0: return

    # mark_pos = world_state.their_robots[mark_robot_id]
    mark_pos = world_state.field.center_field_loc
    print(mark_pos)

    #Finds the line from the ball to the mark position and creates a line between them
    #removing the overlap with the ball on one side and robot on the other
    #This assumes even with mark position parameter that there is a robot there to avoid
    mark_line_dir = np.linalg.norm(ball_pos - mark_pos)
    start_pos = ball_pos - mark_line_dir * constants.Ball.Radius
    end_pos = mark_pos + mark_line_dir * 2.0 * constants.Robot.Radius

    ratio_pt = ((end_pos - start_pos) * ratio) + start_pos
    print(ratio_pt)
    return ratio_pt

    """
    # TODO: change name of Segment's param from "pt" to "pts"
    ball_mark_line = Segment(pt=[
        Point(x=start_pos[0], y=start_pos[1]),
        Point(x=end_pos[0], y=end_pos[1])
    ])
    """


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

        # TODO: use mark_heuristic & CostBehavior to handle marking, rather than having tactic give target_point
        # argument: mark_heuristic: Callable[[np.array], float]
        # > self.mark_heuristic = mark_heuristic

        # workaround for non-working CostBehavior: 
        # initialize move action, update target point every tick (target point being opponent robot pos)

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
        if world_state and world_state.ball.visible:
            # TODO: update w/ robot id (or remove that param from mark_cost)
            self.target_point = get_mark_point(world_state, -1)
            # self.target_point = world_state.ball.pos
        actions = self.root.tick_once(robot, world_state)
        return actions

    def is_done(self, world_state):
        return self.move.is_done(world_state)
