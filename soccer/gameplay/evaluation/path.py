import main
import robocup
import constants


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
    line = robocup.Segment(start, end)
    iterations = 0
    max_iterations = 10

    # While there is a robot in the way
    blocking_robot = find_intersecting_robot(line, blocking_robots, dodge_dist)
    while (blocking_robot is not None) and (iterations < max_iterations):
        # Find next point
        # Next point is +-dodge_dist * perp_vect
        robot_vec = (blocking_robot.pos - next_pt)
        perp_vec = robot_vec.perp_cw().normalized()

        pt1 = perp_vec * dodge_dist + blocking_robot.pos - next_pt
        pt2 = perp_vec * -dodge_dist + blocking_robot.pos - next_pt

        # Find shortest path
        if (pt1.mag() < pt2.mag()):
            next_pt = pt1
        else:
            next_pt = pt2

        # Add dist to total
        total += (next_pt - prev_pt).mag()

        prev_pt = next_pt
        line = robocup.Segment(next_pt, end)
        blocking_robot = find_intersecting_robot(line, blocking_robots,
                                                 dodge_dist)
        iterations += 1

    total += (end - next_pt).mag()

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
def can_collect_ball_before_opponent(our_robots_to_check=None,
                                     their_robots_to_check=None,
                                     our_robots_to_dodge=None,
                                     their_robots_to_dodge=None,
                                     valid_error_percent=0.05):
    if our_robots_to_check is None:
        our_robots_to_check = main.our_robots()

    if their_robots_to_check is None:
        their_robots_to_check = main.their_robots()

    if our_robots_to_dodge is None:
        our_robots_to_dodge = main.our_robots()

    if their_robots_to_dodge is None:
        their_robots_to_dodge = main.their_robots()

    shortest_opp_time = float("inf")
    shortest_our_time = float("inf")
    dodge_dist = constants.Robot.Radius
    closest_robot = None

    # TODO: Do some sort of prediction as the ball moves
    target_pos = main.ball().pos

    # TODO: Take velocity and acceleration into account
    # Find closest opponent robot
    for bot in their_robots_to_check:
        dist = estimate_path_length(bot.pos, target_pos, our_robots_to_dodge,
                                    dodge_dist)
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

    # Greater than 1 when we are further away
    print(shortest_our_time)
    print(shortest_opp_time)
    return shortest_our_time < shortest_opp_time * (1 + valid_error_percent
                                                    ), closest_robot


## Finds the intersecting robots in this line
#  @param line The line to compare the robot locations to
#  @param blocking_robots List of robots to check against the line
#  @param dodge_dist Distance cutoff between the line and robot positions
#  @return The robot (or None) that intersects the line within dodge_dist
def find_intersecting_robot(line, blocking_robots, dodge_dist):
    for bot in blocking_robots:
        if (line.dist_to(bot.pos) < dodge_dist):
            return bot

    return None
