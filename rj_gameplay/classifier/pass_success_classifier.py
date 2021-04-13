import stp.rc as rc
import rj_gameplay.gameplay_node as gameplay_node
import rclpy

import numpy as np
import math as m

def classifier(world_state: rc.WorldState) -> bool:
    our_robots = world_state.our_robots
    opp_robots = world_state.their_robots
    ball = world_state.ball

    # check if any robot on our team has the ball 
    for robot in our_robots:
        if robot.has_ball:
            return True

    for robot in opp_robots:
        if robot.has_ball:
            return False

    net_robot_list = our_robots + opp_robots

    robot = shortest_distance(rc.WorldState, net_robot_list, ball)

    if robot == None:
        return False

    if robot in our_robots:
        return True
    
    return False
    

def shortest_distance(world_state: rc.WorldState, robot_list, ball) -> rc.Robot:
    ball_x = ball.pos()[0]
    ball_y = ball.pos()[1]

    ball_vel_x = ball.vel()[0]
    ball_vel_y = ball.vel()[1]

    minDistance = 100000
    minRobot = None

    for robot in robot_list:
        robot_x = robot.pose()[0]
        robot_y = robot.pose()[1]
        robot_ang = robot.pose()[2]

        dist = distance(robot_x, robot_y, ball_x, ball_y) 

        if (dist < minDistance and sameDirection(robot, ball)):
            minDistance = dist
            minRobot = robot
    
    return robot

def sameDirection(robot, ball) -> bool:
    ball_x = ball.pos()[0]
    ball_y = ball.pos()[1]

    robot_x = robot.pose()[0]
    robot_y = robot.pose()[1]
    robot_ang = robot.pose()[2]

    displacement_x = robot_x - ball_x
    displacement_y = robot_y - ball_y

    ball_ang = m.atan2(displacement_y, displacement_x)
    
    if ball_ang < 0:
        ball_ang = ball_ang + (2 * m.pi)

    if robot_ang < 0:
        robot_ang = robot_ang + (2 * m.pi)
    
    if abs(ball_ang - robot_ang) < (m.pi / 2):
        return True
    else:
        return False

        

def distance(x1, y1, x2, y2) -> int:
    return ((x2 - x1) ** 2) + ((y2 - y1) ** 2)

def test_classifier(world_state: rc.WorldState) -> None:
    classifier(world_state)

play_selector = gameplay_node.EmptyPlaySelector()
gameplay = gameplay_node.GameplayNode(play_selector)

while(True):
    rclpy.spin_once(gameplay_node)
    test_classifier(gameplay_node.get_world_state())
