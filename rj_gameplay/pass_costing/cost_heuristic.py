import numpy as np
import stp.rc as rc
import stp.testing as testing
import rj_gameplay.gameplay_node as gameplay_node
import rclpy
import math

#max parameters for the various cost functions
MAX_ROBOT_DISTANCE = 5
MAX_CLOSEST_OPPONENT_DISTANCE = 6
MIN_DOWNFIELD_DISTANCE = -2
MAX_DOWNFIELD_DISTANCE = 3
robot_distance_weight = 1
closest_opponent_distance_weight = 1
downfield_distance_weight = 0.5

def cost_heuristic(world_state: rc.WorldState, target_robot: rc.Robot) -> float:
    our_robots = world_state.our_robots
    their_robots = world_state.their_robots
    robot_with_ball = our_robots[0]
    for robot in our_robots:
        if robot.ball_sense_triggered:
            robot_with_ball = robot
            if robot_with_ball == target_robot:
                return 99999

    running_cost = 0

    #downfield distance
    running_cost += downfield_distance_weight * \
        normalize_cost(downfield_distance(robot_with_ball, target_robot), \
            lower_bound=MIN_DOWNFIELD_DISTANCE, upper_bound=MAX_DOWNFIELD_DISTANCE)

    # distance from passing robot to potential passer
    running_cost += robot_distance_weight * \
        normalize_cost(robot_distance(robot_with_ball, target_robot), \
            lower_bound=0, upper_bound=MAX_ROBOT_DISTANCE)

    # distance to nearest opponent robot
    running_cost += closest_opponent_distance_weight * \
        normalize_cost(MAX_CLOSEST_OPPONENT_DISTANCE - closest_opponent_robot_distance(target_robot, their_robots), \
        lower_bound=0, upper_bound=MAX_CLOSEST_OPPONENT_DISTANCE)

    return running_cost

# downfield distance
def downfield_distance(passer: rc.Robot, receiver: rc.Robot):
    return passer.pose[1] - receiver.pose[1]

# distance from passing robot to potential passer
def robot_distance(first_robot: rc.Robot, second_robot: rc.Robot) -> float:
    return math.sqrt(pow(first_robot.pose[0] - second_robot.pose[0], 2)
        + pow(first_robot.pose[1] - second_robot.pose[1], 2))

# distance to nearest opponent robot
def closest_opponent_robot_distance(our_robot: rc.Robot, their_robots: list) -> float:
    min_distance = 99999
    for their_robot in their_robots:
        distance = robot_distance(our_robot, their_robot)
        if distance < min_distance:
            min_distance = distance
    return min_distance

#returns a value between 0 and 1 that is proportional to how close the cost is to the upper bound
def normalize_cost(cost: float, lower_bound: float, upper_bound: float) -> float:
    if cost < lower_bound:
        return 0
    if cost > upper_bound:
        return 1
    # could change normalization later to be more like a normal curve
    range = upper_bound - lower_bound
    cost -= lower_bound
    return cost / range