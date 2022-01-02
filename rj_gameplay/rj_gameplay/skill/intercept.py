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

import stp.global_parameters as global_parameters
SLOWNESS_FACTOR = 0.5

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
        intercept_pt = self.get_intercept_pt(ball_pos, ball_vel, robot.pose[:2])

        self.move.target_point = intercept_pt
        self.move.face_point = ball_pos

        return self.move.tick(robot, world_state, intent)


    def get_intercept_pt(self, ball_pos, ball_vel, robot_pos: np.ndarray) -> np.ndarray:
        # TODO: put into motion planning?
        """
        Finds optimal intercept point with law of cosines. Assumes constant velocity of robot, so slowness factor applied to robot's max speed.

        Credit: https://www.codeproject.com/Articles/990452/Interception-of-Two-Moving-Objects-in-D-Space
        """

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

        # compute time to intercept (see Credit for explanation)
        ball_speed = np.linalg.norm(ball_vel)
        robot_speed = SLOWNESS_FACTOR * global_parameters.soccer.robot.max_speed
        ball_to_robot_dist = robot_pos - ball_pos
        
        a = ball_speed ** 2 + robot_speed ** 2
        b = 2 * np.dot(ball_to_robot_dist, ball_vel)
        c = np.linalg.norm(ball_to_robot_dist)

        roots = np.roots([a, b, c])
        times_to_intercept = np.where(roots > 0, roots, np.inf)
        if times_to_intercept.shape[0] == 0:
            # if no positive roots, no way to intercept ball
            return ball_pos
        min_time = np.min(times_to_intercept)
        
        # given time, compute point of intercept (see Credit for explanation)
        # could also compute robot_velocity here, but that assumes const vel
        intercept_pt = ball_pos + ball_vel * min_time

        return intercept_pt

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
