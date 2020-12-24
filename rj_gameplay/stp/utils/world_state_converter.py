from rj_msgs import msg
import stp.rc as rc
import numpy as np

from typing import List

def robotstate_to_robot(robot_msg: msg._robot_state.RobotState, index: int) -> rc.Robot:
    """
        :return: robot class representing the state of the robot.
    """
    robot_id = index

    x = robot_msg.pose.position.x
    y = robot_msg.pose.position.y
    theta = robot_msg.pose.heading
    pose = np.array([x,y,theta])

    dx = robot_msg.velocity.linear.x
    dy = robot_msg.velocity.linear.y
    dtheta = robot_msg.velocity.angular
    twist = np.array([dx,dy,dtheta])

    has_ball: bool
    has_ball = None

    robot = rc.Robot(robot_id, pose, twist, has_ball)

    return robot

def ballstate_to_ball(ball_msg: msg._ball_state.BallState) -> rc.Ball:
    """
        :return: ball class representing the state of the ball.
    """
    x = ball_msg.position.x
    y = ball_msg.position.y
    pos = np.array([x,y])

    dx = ball_msg.velocity.x
    dy = ball_msg.velocity.y
    vel = np.array([x,y])

    ball = rc.Ball(pos,vel)

    return ball

def worldstate_message_converter(msg: msg.WorldState) -> rc.WorldState:
    """
        :return: world state class representing the state of the world.
    """
    our_robots: List[rc.Robot]
    our_robots = []

    their_robots: List[rc.Robot]
    their_robots = []

    for i in range(len(msg.our_robots)):
        our_robots.append(robotstate_to_robot(msg.our_robots[i], i))

    for i in range(len(msg.their_robots)):
        our_robots.append(robotstate_to_robot(msg.their_robots[i], i))

    ball = ballstate_to_ball(msg.ball)

    world_state = rc.WorldState(our_robots, their_robots, ball)

    return world_state