from abc import ABC, abstractmethod

import rj_gameplay.eval as eval
import argparse
import py_trees
import sys
import time

import stp.skill as skill
import stp.role as role
import stp.action as action
# TODO: after fixing intercept planner, use that below
#       (go back to this version of the code)
# from rj_gameplay.action import intercept
from stp.skill.action_behavior import ActionBehavior
import stp.rc as rc
import numpy as np
from rj_gameplay.skill import move
from rj_msgs.msg import RobotIntent

class Intercept(skill.ISkill):
    def __init__(self, robot: rc.Robot = None):
        self.robot = robot
        self.block_pt = None
        self.move = move.Move(robot)
        # self.move.target_vel = np.array([0.0, 0.0])

    def tick(self, robot: rc.Robot, world_state: rc.WorldState, intent: RobotIntent):
        # Intercept is init with None, so it has to be properly set in tick()
        # TODO: make this explicit in init by setting fields to None
        self.robot = robot
        self.move.robot = robot

        ball_pos = world_state.ball.pos
        ball_vel = world_state.ball.vel
        block_pt = self.get_block_pt(ball_pos, ball_vel, robot.pose[:2])

        print(block_pt)
        # CRASH = 1 / 0

        self.move.target_point = block_pt
        self.move.face_point = ball_pos

        return self.move.tick(robot, world_state, intent)

    # TODO: this method is ripped from goalie_tactic, either make goalie use Intercept or have common import
    # actually modifying it so it's not identical to goalie one (merge?)
    def get_block_pt(self, ball_pos, ball_vel, robot_pos: np.ndarray) -> np.ndarray:

        # ball_dir = ball_vel / (np.linalg.norm(ball_vel) + 1e-9)
        # 1) set ball_pos as origin of coordinate frame
        # robot_pos -= ball_pos

        # 2) find projection of robot_pos onto ball_vel
        proj = np.dot(robot_pos, ball_vel) / (ball_vel + 1e-9)

        # 3) scale proj back to real coordinates, return
        # block_pt = proj + ball_pos
        block_pt = proj

        """
        tangent = vel / (np.linalg.norm(vel) + 1e-6)

        # Find out where it would cross
        time_to_cross = np.abs(pos[1] / vel[1]) if np.abs(vel[1]) > 1e-6 else 0
        cross_x = pos[0] + vel[0] * time_to_cross
        cross_position = np.array([np.clip(cross_x, a_min=-0.6, a_max=0.6), 0.0])

        tangent = cross_position - pos
        tangent /= np.linalg.norm(tangent)
        block_pt = np.dot(tangent, pos - my_pos) * tangent + pos
        """

        return block_pt

    def is_done(self, world_state) -> bool:
        # TODO: resolve these shared params
        SETTLE_BALL_SPEED_THRESHOLD = 1.0
        SETTLE_BALL_DIST_THRESHOLD = 1.0

        # TODO: make dist() util
        speed_is_slow = np.linalg.norm(world_state.ball.vel) < SETTLE_BALL_SPEED_THRESHOLD
        ball_is_close = np.linalg.norm(self.robot.pose[0:2] - world_state.ball.pos[0:2]) < SETTLE_BALL_DIST_THRESHOLD

        print("int speed")
        print(np.linalg.norm(world_state.ball.vel))
        print(speed_is_slow)
        if speed_is_slow:
            return True
        return False

        # NO because move needs to be replanned
        # return self.move.is_done(world_state)

    def __str__(self):
        return f"Intercept(robot={self.robot.id if self.robot is not None else '??'})"
