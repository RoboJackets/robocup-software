import constants
import robocup
import main


## Find the chance of a shot succeeding by looking at pass distance and what robots are in the way
#  The total goal angle as well as the  percent covered  is taken into account
# @param from_point The Point the shot is coming from
# @param excluded_robots A list of robots that shouldn't be counted as obstacles to this shot
# @return a value from zero to one that estimates the probability of the shot succeeding
def eval_shot(from_point, excluded_robots=[]):
    kick_eval = robocup.KickEvaluator(main.system_state())
    for r in excluded_robots:
        kick_eval.add_excluded_robot(r)
    point, chance = kick_eval.eval_pt_to_opp_goal(from_point)
    return chance

## Shoot through a formation of enemy robots at a target
#
# @param target_pos: the target to shoot at
# @param max max_shooting_angle: The largest angle we will search to find a gap
# @param robot_offset: Max angle offset from an enemy robot we will shoot
# @return a point
def find_gap(target_pos=constants.Field.TheirGoalSegment.center(), max_shooting_angle=60, robot_offset=8, dist_from_point=.75):
    if (not main.ball().valid):
        return target_pos

    # Find the hole in the defenders to kick at
    # The limit is 20 cm so any point past it should be defenders right there
    win_eval = robocup.WindowEvaluator(main.context())

    # 500 cm min circle distance plus the robot width
    test_distance = dist_from_point + constants.Robot.Radius

    # +- max offset to dodge ball
    max_angle = max_shooting_angle * constants.DegreesToRadians

    # How much left and right of a robot to give
    # Dont make this too big or it will always go far to the right or left of the robots
    robot_angle_offset = robot_offset * constants.DegreesToRadians

    zero_point = robocup.Point(0, 0)

    # Limit the angle so as we get closer, we dont miss the goal completely as much
    goal_vector = target_pos - main.ball().pos
    max_length_vector = robocup.Point(constants.Field.Length, constants.Field.Width)
    goal_limit = (goal_vector.mag() / max_length_vector.mag()) * max_angle

    # Limit on one side so we dont directly kick out of bounds
    # Add in the angle from the sideline to the target
    field_limit = (1 - abs(main.ball().pos.x) / (constants.Field.Width / 2)) * max_angle
    field_limit = field_limit + goal_vector.angle_between(robocup.Point(0, 1))

    # Limit the angle based on the opponent robots to try and always minimize the
    left_robot_limit = 0
    right_robot_limit = 0

    for robot in main.their_robots():
        ball_to_bot = robot.pos - main.ball().pos

        # Add an extra radius as wiggle room
        # kick eval already deals with the wiggle room so it isn't needed there
        if (ball_to_bot.mag() <= test_distance + constants.Robot.Radius):
            angle = goal_vector.angle_between(ball_to_bot)

            # Try and rotate onto the goal vector
            # if we actually do, then the robot is to the right of the ball vector
            ball_to_bot.rotate(zero_point, angle)
            if (ball_to_bot.angle_between(goal_vector) < 0.01):
                right_robot_limit = max(right_robot_limit, angle + robot_angle_offset)
            else:
                left_robot_limit = max(left_robot_limit, angle + robot_angle_offset)
        else:
            win_eval.add_excluded_robot(robot)


    # Angle limit on each side of the bot->goal vector
    left_angle = max_angle
    right_angle = max_angle

    # Make sure we limit the correct side due to the field
    if main.ball().pos.x < 0:
        left_angle = min(left_angle, field_limit)
    else:
        right_angle = min(right_angle, field_limit)

    # Limit due to goal
    left_angle = min(left_angle, goal_limit)
    right_angle = min(right_angle, goal_limit)

    # Limit to just over the robots
    if (left_robot_limit is not 0):
        left_angle = min(left_angle, left_robot_limit)
    if (right_robot_limit is not 0):
        right_angle = min(right_angle, right_robot_limit)

    # Get the angle that we need to rotate the target angle behind the defenders
    # since kick eval doesn't support a nonsymmetric angle around a target
    rotate_target_angle = (left_angle + -right_angle)/2
    target_width = (left_angle + right_angle)

    target_point = goal_vector.normalized() * test_distance
    target_point.rotate(zero_point, rotate_target_angle)

    windows, window = win_eval.eval_pt_to_pt(main.ball().pos, target_point + main.ball().pos, target_width)

    # Test draw points
    target_point.rotate(zero_point, target_width/2)
    p1 = target_point + main.ball().pos
    target_point.rotate(zero_point, -target_width)
    p2 = target_point + main.ball().pos
    p3 = main.ball().pos
    main.debug_drawer().draw_polygon([p1, p2, p3], (0, 0, 255),
                                     "Free Kick search zone")


    is_opponent_blocking = False

    for robot in main.their_robots():
        if (goal_vector.dist_to(robot.pos) < constants.Robot.Radius and
            (main.ball().pos - robot.pos).mag() < test_distance):
            is_opponent_blocking = True

    # Vector from ball position to the goal
    ideal_shot = (target_pos - main.ball().pos).normalized()

    # If on our side of the field and there are enemy robots around us,
    # prioritize passing forward vs passing towards their goal
    # Would have to change this if we are not aiming for their goal
    if main.ball().pos.y < constants.Field.Length / 2 and len(windows) > 1:
        ideal_shot = robocup.Point(0, 1)

    main.debug_drawer().draw_line(
        robocup.Line(main.ball().pos, target_pos), (0, 255, 0), "Target Point")

    # Weights for determining best shot
    k1 = 1.5 # Weight of closeness to ideal shot
    k2 = 1 # Weight of shot chance

    # Iterate through all possible windows to find the best possible shot
    if windows:
        best_shot = window.segment.center()
        best_weight = 0
        for wind in windows:
            pos_to_wind = (wind.segment.center() - main.ball().pos).normalized()
            dot_prod = pos_to_wind.dot(ideal_shot)
            weight = k1 * dot_prod + k2 * wind.shot_success
            if weight > best_weight:
                best_weight = weight
                best_shot = wind.segment.center()

        main.debug_drawer().draw_line(
            robocup.Line(main.ball().pos, best_shot), (255, 255, 0),
            "Target Shot")

        best_shot = robocup.Point(0,1) + main.ball().pos
        return best_shot
    else:
        return constants.Field.TheirGoalSegment.center()
