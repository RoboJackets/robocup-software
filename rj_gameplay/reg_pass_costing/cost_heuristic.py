import stp.rc as rc
import rj_gameplay.gameplay_node as gameplay_node
import rclpy
import numpy as np

MAX_PASS_DIST = 5
MAX_GOAL_DIST = 7.5
MAX_AVG_OPP_DIST = 6
MAX_NEAREST_OPP_DIST = 2.5
MAX_OPP_BETWEEN_ROBOT_GOAL = 5
MAX_OPP_BETWEEN_ROBOT_BALL = 1
MIN_GOAL_DIST_BONUS = 1
MAX_GOAL_DIST_BONUS = 1.75

GOAL_DIST_COST = 150
AVG_OPP_DIST_COST = 75
NEAREST_OPP_DIST_COST = 300
OPP_BETWEEN_ROBOT_GOAL_COST = 325
OPP_BETWEEN_ROBOT_BALL_COST = 650

MAX_COST = 1000

def cost_heuristic(world_state: rc.WorldState, robot: rc.Robot) -> float:

    field = world_state.field

    our_robots = world_state.our_robots
    their_robots = world_state.their_robots

    robot_with_ball = our_robots[0]
    for bot in our_robots:
        if bot.ball_sense_triggered:
            robot_with_ball = bot
            if robot_with_ball == robot:
                return -1

    # return 1 if robot is too far awat from robot_with_ball
    if (distance_from_ball(robot, robot_with_ball) > MAX_PASS_DIST):
        return 1 

    running_cost = 0
    running_cost += distance_from_goal(robot, robot_with_ball, field)
    running_cost += average_opponent_distance(robot, their_robots)
    running_cost += nearest_opponent_distance(robot, their_robots)
    running_cost += opponents_between_robot_goal(robot, their_robots, field)
    running_cost += opponents_between_robot_ball(robot, their_robots, robot_with_ball)

    if running_cost >= MAX_COST:
        return 1
    return round(running_cost / MAX_COST, 3)

# returns distance between the robot and robot_with_ball
def distance_from_ball(robot: rc.Robot, robot_with_ball: rc.Robot) -> float:
    robot_pos = robot.pose[0:2]
    robot_with_ball_pos = robot_with_ball.pose[0:2]
    return np.linalg.norm(robot_with_ball_pos - robot_pos)

# returns whether or not the opponent robot can intercept pass/path to goal
def intercept_calc(A: np.array, B: np.array, E: np.array, target_dist: float) -> bool:
    AB_BE = np.dot(B - A, E - B)
    AB_AE = np.dot(B - A, E - A)

    if AB_BE < 0 and AB_AE > 0:
        intercept_dist = abs(np.cross(B - A, E - A)) / np.linalg.norm(B - A)
        if intercept_dist < target_dist / 6:
            return True
    return False

# returns the cost for the robot's distance from goal
def distance_from_goal(robot: rc.Robot, robot_with_ball: rc.Robot, field: rc.Field) -> float:
    robot_pos = robot.pose[0:2]
    opp_goal_pos = field.their_goal_loc
    goal_dist = np.linalg.norm(opp_goal_pos - robot_pos)

    if goal_dist > MAX_GOAL_DIST:
        goal_dist = MAX_GOAL_DIST
    return (goal_dist / MAX_GOAL_DIST) * GOAL_DIST_COST * distance_from_goal_bonus(robot, robot_with_ball)

# returns the multiplier bonus for the robot's distance from goal based on robot positioning
def distance_from_goal_bonus(robot: rc.Robot, robot_with_ball: rc.Robot) -> float:
    robot_Ypos = robot.pose[1]
    robot_with_ball_Ypos = robot_with_ball.pose[1]
    bonus = 1

    BALL_MIN_yPOS = 3
    BALL_MAX_yPOS = 5.5
    if robot_with_ball_Ypos >= BALL_MIN_yPOS:
        if robot_with_ball_Ypos > BALL_MAX_yPOS:
            robot_with_ball_Ypos = BALL_MAX_yPOS
        bonus *= ((MAX_GOAL_DIST_BONUS - MIN_GOAL_DIST_BONUS) * (robot_with_ball_Ypos - BALL_MIN_yPOS) / (BALL_MAX_yPOS - BALL_MIN_yPOS)) + 1

    ROBOT_MAX_yPOS = 2.5
    if robot_Ypos < ROBOT_MAX_yPOS:
        bonus *= 1.5

    return bonus

# returns the cost for the robot's average distance from opponent robots
def average_opponent_distance(robot: rc.Robot, their_robots: list) -> float:
    robot_pos = robot.pose[0:2]
    total_opp_dist = 0
    for their_robot in their_robots:
        their_robot_pos = their_robot.pose[0:2]
        curr_opp_dist = np.linalg.norm(their_robot_pos - robot_pos)
        total_opp_dist += curr_opp_dist

    avg_opp_dist = total_opp_dist / len(their_robots)

    if avg_opp_dist > MAX_AVG_OPP_DIST:
        avg_opp_dist = MAX_AVG_OPP_DIST
    return ((MAX_AVG_OPP_DIST - avg_opp_dist) / MAX_AVG_OPP_DIST) * AVG_OPP_DIST_COST

# returns the cost for the robot's distance from the nearest opponent robot
def nearest_opponent_distance(robot: rc.Robot, their_robots: list) -> float:
    robot_pos = robot.pose[0:2]
    nearest_opp_dist = float("inf")
    for their_robot in their_robots:
        their_robot_pos = their_robot.pose[0:2]
        curr_opp_dist = np.linalg.norm(their_robot_pos - robot_pos)
        if curr_opp_dist < nearest_opp_dist:
            nearest_opp_dist = curr_opp_dist

    if nearest_opp_dist > MAX_NEAREST_OPP_DIST:
        nearest_opp_dist = MAX_NEAREST_OPP_DIST
    return ((MAX_NEAREST_OPP_DIST - nearest_opp_dist) / MAX_NEAREST_OPP_DIST) * NEAREST_OPP_DIST_COST

# returns the cost for the number of opponent robots between the robot and the opponent goal
def opponents_between_robot_goal(robot: rc.Robot, their_robots: list, field: rc.Field) -> float:
    robot_pos = robot.pose[0:2]
    opp_goal_pos = field.their_goal_loc
    goal_dist = np.linalg.norm(opp_goal_pos - robot_pos)

    opp_between_robot_goal = 0
    for their_robot in their_robots:
        their_robot_pos = their_robot.pose[0:2]
        if intercept_calc(robot_pos, opp_goal_pos, their_robot_pos, goal_dist):
            opp_between_robot_goal += 1

    if opp_between_robot_goal > MAX_OPP_BETWEEN_ROBOT_GOAL:
            opp_between_robot_goal = MAX_OPP_BETWEEN_ROBOT_GOAL
    return (opp_between_robot_goal / MAX_OPP_BETWEEN_ROBOT_GOAL) * OPP_BETWEEN_ROBOT_GOAL_COST

# returns the cost for the number of opponent robots between the robot and the robot_with_ball
def opponents_between_robot_ball(robot: rc.Robot, their_robots: list, robot_with_ball: rc.Robot) -> float:
    robot_pos = robot.pose[0:2]
    robot_with_ball_pos = robot_with_ball.pose[0:2]
    ball_dist = np.linalg.norm(robot_with_ball_pos - robot_pos)

    opp_between_robot_ball = 0
    for their_robot in their_robots:
        their_robot_pos = their_robot.pose[0:2]
        if intercept_calc(robot_with_ball_pos, robot_pos, their_robot_pos, ball_dist):
            opp_between_robot_ball += 1

    if opp_between_robot_ball > MAX_OPP_BETWEEN_ROBOT_BALL:
            opp_between_robot_ball = MAX_OPP_BETWEEN_ROBOT_BALL
    return (opp_between_robot_ball / MAX_OPP_BETWEEN_ROBOT_BALL) * OPP_BETWEEN_ROBOT_BALL_COST
