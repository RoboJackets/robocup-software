import numpy as np
import stp.rc as rc
import stp.testing as testing
import rj_gameplay.gameplay_node as gameplay_node
import rclpy
import math

#max parameters for the various cost functions
MAX_ROBOT_DISTANCE = 5

def test_cost(world_state: rc.WorldState, target_robot: rc.Robot) -> float:
    our_robots = world_state.our_robots
    their_robots = world_state.their_robots
    robot_with_ball = our_robots[0]
    for robot in our_robots:
        if robot.ball_sense_triggered:
            robot_with_ball = robot

    running_cost = 0
    running_cost += normalize_cost(robot_distance(robot_with_ball, target_robot), 0, MAX_ROBOT_DISTANCE)
    running_cost += normalize_cost(6 - closest_opponent_robot_distance(target_robot, their_robots), 0, 6)
    return running_cost

def robot_distance(first_robot: rc.Robot, second_robot: rc.Robot) -> float:
    return math.sqrt(pow(first_robot.pose[0] - second_robot.pose[0], 2)
        + pow(first_robot.pose[1] - second_robot.pose[1], 2))

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

def main():
    np.ndarray(shape=(3,1))
    our_robots = []
    their_robots = []
    robot_count = 6
    for i in range(robot_count):
        our_robot = rc.Robot(robot_id=i,
                is_ours=True,
                pose=np.array([i,i,0]),
                twist=np.array([0,0,0]),
                ball_sense_triggered= i == 0,
                visible=True,
                has_ball_sense=True,
                kicker_charged=True,
                kicker_healthy=True,
                lethal_fault=False)
        their_robot = rc.Robot(robot_id=i+robot_count,
                is_ours=True,
                pose=np.array([i,-i,0]),
                twist=np.array([0,0,0]),
                ball_sense_triggered= False,
                visible=True,
                has_ball_sense=True,
                kicker_charged=True,
                kicker_healthy=True,
                lethal_fault=False)
        our_robots.append(our_robot)
        their_robots.append(their_robot)
    world_state = rc.WorldState(our_robots, their_robots, None, None, None)
    for i in world_state.our_robots:
        print(test_cost(world_state, i))

if __name__ == "__main__":
    main()