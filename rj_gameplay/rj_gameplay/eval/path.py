import sys
sys.path.insert(1, "../../stp")
import rc
from typing import List, Optional, Tuple


## Estimates the length of a path given a start and end point
#  @param start The start point of the path
#  @param end The end point of the path
#  @param blocking_robots The list of robots to dodge
#  @param dodge_dist The amount of distance to dodge left or right to miss a robot
#  @return the distance from start to end
def estimate_path_length(start, end, blocking_robots, dodge_dist):
    total = 0
    next_pt = start
    prev_pt = start
    line = [start, end]
    iterations = 0
    max_iterations = 10

    # While there is a robot in the way
    blocking_robot = find_intersecting_robot(line, blocking_robots, dodge_dist)
    while (blocking_robot is not None) and (iterations < max_iterations):
        # Find next point
        # Next point is +-dodge_dist * perp_vect
        robot_vec = (blocking_robot.pos[:-1] - next_pt)
        perp = [-robot_vec[1], robot_vec[0]]
        pn = sqrt(perp[0]**2 + perp[1]**2)
        perp_vec = perp / pn

        pt1 = perp_vec * dodge_dist + blocking_robot.pos[:-1] - next_pt
        pt2 = perp_vec * -dodge_dist + blocking_robot.pos[:-1] - next_pt
        m1 = sqrt(pt1[0]**2 + pt1[1]**2)
        m2 = sqrt(pt2[0]**2 + pt2[1]**2)

        # Find shortest path
        if (m1 < m2):
            next_pt = pt1
        else:
            next_pt = pt2

        # Add dist to total
        t = (next_pt - prev_pt)
        total += sqrt(t[0]**2 + t[1]**2)

        prev_pt = next_pt
        line = [next_pt, end]
        blocking_robot = find_intersecting_robot(line, blocking_robots,
                                                 dodge_dist)
        iterations += 1

    tx = (end - next_pt) 
    total += sqrt(tx[0]**2 + tx[1]**2)

    return total


## Whether any robot can collect the ball before the opponent
#  @param our_robots_to_check List of our robots that can move to ball
#  @param their_robots_to_check List of their robots that can move to ball
#  @param our_robots_to_dodge List of our robots that can be considered obsticles to dodge
#  @param their_Robots_to_dodge List of their robots that can be considered obsticles to dodge
#  @param valid_error_percent Wiggle room so if the path is slightly off, it still tries if it is close
#  @return Tuple
#       Whether we can collect the ball before the opponen
#       The closest robot on our team
#  @note If any imputs are None, their values are defaulted
def can_collect_ball_before_opponent(ball, their_robots, our_robots, our_robots_to_check, their_robots_to_check, our_robots_to_dodge, their_robots_to_dodge, valid_error_percent):
    if our_robots_to_check is None:
        our_robots_to_check = our_robots

    if their_robots_to_check is None:
        their_robots_to_check = their_robots

    if our_robots_to_dodge is None:
        our_robots_to_dodge = our_robots

    if their_robots_to_dodge is None:
        their_robots_to_dodge = their_robots

    shortest_opp_time = float("inf")
    shortest_our_time = float("inf")
    dodge_dist = 0.09
    closest_robot = None

    # TODO: Do some sort of prediction as the ball moves
    target_pos = ball.pos

    # TODO: Take velocity and acceleration into account
    # Find closest opponent robot
    for bot in their_robots_to_check:
        dist = estimate_path_length(bot.pos, target_pos, our_robots_to_dodge,
                                    dodge_dist)
        td = (target_pos - bot.pos)
        target_dir = (target_pos - bot.pos).normalized()
        time = robocup.get_trapezoidal_time(dist, dist, 2.2, 1,
                                            target_dir.dot(bot.vel) /
                                            target_dir.mag(), 0)
        if (time < shortest_opp_time):
            shortest_opp_time = time

    # Find closest robot on our team
    for bot in our_robots_to_check:
        dist = estimate_path_length(bot.pos, target_pos, their_robots_to_dodge,
                                    dodge_dist)
        target_dir = (target_pos - bot.pos).normalized()
        time = robocup.get_trapezoidal_time(dist, dist, 2.2, 1,
                                            target_dir.dot(bot.vel) /
                                            target_dir.mag(), 0)
        if (time < shortest_our_time):
            shortest_our_time = time
            closest_robot = bot

    return shortest_our_time < shortest_opp_time * (1 + valid_error_percent
                                                    ), closest_robot


## Finds the intersecting robots in this line
#  @param line The line to compare the robot locations to
#  @param blocking_robots List of robots to check against the line
#  @param dodge_dist Distance cutoff between the line and robot positions
#  @return The robot (or None) that intersects the line within dodge_dist
def find_intersecting_robot(line: robocup.Segment,
                            blocking_robots: List[robocup.Robot],
                            dodge_dist: float) -> Optional[robocup.Robot]:
    for bot in blocking_robots:
        if (line.dist_to(bot.pos) < dodge_dist):
            return bot

    return None
