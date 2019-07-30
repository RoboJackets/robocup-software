import main
import robocup
import math
import constants
import evaluation.ball


## Predicts the impending kick direction based on the orientation of the robot and the
#  angle of approach
#
# @param robot: The robot which we want to estimate
# @return Angle of most likely kick
def predict_kick_direction(robot):
    angle = robot.angle
    pos = robot.pos
    angle_vel = robot.angle_vel

    # Use distance from bot to ball to predict time it takes to intercept
    # Calculates direct robot to future ball position
    inst_ball_time = evaluation.ball.time_to_ball(robot)

    future_ball_pos = main.ball().predict_pos(inst_ball_time)
    direction = (future_ball_pos - pos).normalized()
    ball_angle_predict = direction.angle()

    # Predict the robot direction based on angular velocity and angle
    robot_angle_predict = angle + angle_vel * inst_ball_time

    filter_coeff = 0.7
    return filter_coeff * robot_angle_predict + \
           (1 - filter_coeff) * ball_angle_predict


def get_points_from_rect(rect, step=0.5):
    outlist = []
    currentx = rect.min_x()
    currenty = rect.min_y()

    while currenty <= rect.max_y():
        while currentx <= rect.max_x():
            if constants.Field.OurGoalZoneShape.contains_point(robocup.Point(
                    currentx, currenty)):
                currentx += step
                continue

            outlist.extend([robocup.Point(currentx, currenty)])

            currentx += step
        currenty += step
        currentx = rect.min_x()

    return outlist


## Creates a zone that may cause a risk in the future
#  Based off the...
#   Risk score in that position
#   Adviailbity of opponent robots to reach that point
#   Space in that area
#
# @param ignore_robots: Ignore these robots in defensive calculations
# @return Returns the best position to cover an error
def create_area_defense_zones(ignore_robots=[]):
    # Create a 2D list [N][M] where N is the bucket
    # and M is the index along that point
    # The lists contains (robocup.Point, score)
    points = [[]]

    # Amnt each bucket holds
    angle_inc = math.pi / 10
    # Amnt to inc each value by
    dist_inc = 2.5

    # Holds the float angle (Radians)
    angle = 0
    # Holds the integer bucket based on angle
    angle_cnt = 0
    # Holds dist
    dist = dist_inc

    score_sum = 0
    point_cnt = 0

    # Populates all the buckets with values
    while angle < 2 * math.pi:

        point = main.ball().pos
        while constants.Field.FieldRect.contains_point(point):
            x = math.cos(angle) * dist + main.ball().pos.x
            y = math.sin(angle) * dist + main.ball().pos.y
            point = robocup.Point(x, y)
            score = estimate_risk_score(point, ignore_robots)

            # Add into bucketed list
            points[angle_cnt].extend([(point, score)])

            # Keep track of all the big stuff for later
            score_sum += score
            point_cnt += 1

            dist += dist_inc

        points.extend([[]])
        angle_cnt += 1
        angle += angle_inc
        dist = dist_inc

    # Bot outside field
    if (point_cnt == 0):
        return None

    avg = score_sum / point_cnt
    largest_bucket = 0

    # Removes any point-scores that are under the avg in all the buckets
    # Finds the bucket with the most values above avg
    for i in range(angle_cnt):
        points[i] = list(filter(lambda point_score: point_score[1] > avg,
                                points[i]))

        if len(points[i]) > len(points[largest_bucket]):
            largest_bucket = i

    # Max amount to go in either direction (Total area covers ~PI/4)
    max_bucket_dist = round(1 / angle_inc * math.pi / 2)
    # Min amount in bucket to be counted in terms of % of max
    min_bucket_amnt = 0.25 * len(points[largest_bucket])

    # Move a max of X buckets in either direction
    # where the number of buckets is > N and 
    # the number of buckets is decreasing a certain amount
    # based on how far away it is
    bucket_list = []
    bucket_pt_sum = robocup.Point(0, 0)
    bucket_score_sum = 0

    for i in range(-largest_bucket - 1, max_bucket_dist + 1):
        index = (largest_bucket + i) % angle_cnt
        if len(points[index]) > min_bucket_amnt:
            bucket_list.extend(points[index])

    # Do a psudo-density centroid calculation to find where to defend
    for point in bucket_list:
        bucket_pt_sum += point[0] * point[1]**2
        bucket_score_sum += point[1]**2

    return bucket_pt_sum / bucket_score_sum


## Estimates how dangerous an enemy robot can be at a certain point
#  Takes pass / shot and time to execute on ball into account
#
# @param pos: Position in which to estimate score at
# @return Risk score at that point
def estimate_risk_score(pos, ignore_robots=[]):
    # Caches some kick eval functions
    max_time = 1
    max_ball_vel = 8  # m/s per the rules
    est_turn_vel = 8  # rad/s per a random dice roll (Over estimates oppnents abilities)

    kick_eval = robocup.KickEvaluator(main.system_state())

    for r in ignore_robots:
        kick_eval.add_excluded_robot(r)

    _, pass_score = kick_eval.eval_pt_to_robot(main.ball().pos, pos)
    shot_pt, shot_score = kick_eval.eval_pt_to_our_goal(pos)

    # Dist to ball
    ball_pos_vec = pos - main.ball().pos
    dist = ball_pos_vec.mag()
    max_dist = robocup.Point(constants.Field.Width,
                             constants.Field.Length).mag()

    # Closest opp robot
    closest_opp_bot = evaluation.opponent.get_closest_opponent(main.ball().pos)
    delta_angle = ball_pos_vec.angle() - \
                  predict_kick_direction(closest_opp_bot)
    delta_angle = math.atan2(math.sin(delta_angle), math.cos(delta_angle))

    # Underestimates max time to execute on ball
    # Assumes perfect opponent
    time = dist / max_ball_vel + math.fabs(delta_angle) / est_turn_vel
    # Limits to max time so we can invert it later on
    time = min(time, max_time)

    # Center, Dist, Angle
    pos_score = evaluation.field.field_pos_coeff_at_pos(pos, 0.05, 0.3, 0.05,
                                                        False)
    space_coeff = evaluation.field.space_coeff_at_pos(pos, ignore_robots,
                                                      main.our_robots())

    # Delta angle between pass recieve and shot
    delta_angle = ball_pos_vec.angle() - (shot_pt - pos).angle()
    delta_angle = math.atan2(math.sin(delta_angle), math.cos(delta_angle))
    angle_coeff = math.fabs(delta_angle) / math.pi

    # Shot only matters if its a good pass
    # Add pass back in for checking if pass is good (while shot is not)
    #
    # Add in time to weight closer points higher
    #
    # Pos is weighted higher to remove bad positions from calculations
    #
    # Space is weighted in so it is weighted towards lower density areas
    #
    # Delta angle for shot is weighted in so easier shots are weighted higher
    #
    # Distance to the ball squared

    #  Pass, Time, Pos, Space, Angle
    weights = [0.1, 0.1, 2, 0.4, 0.3, 1]
    score = weights[0] * (shot_score * pass_score + pass_score) / 2 + \
            weights[1] * (max_time - time) + \
            weights[2] * pos_score + \
            weights[3] * (1 - space_coeff) + \
            weights[4] * angle_coeff + \
            weights[5] * (1 - dist / max_dist)**2

    return score / sum(weights)


## Decides where the best positions for defense is
#
# @return area_defense_position, highest_risk_robot, 2nd_highest_risk_robot
def find_defense_positions(ignore_robots=[]):

    their_risk_scores = []

    for bot in main.their_robots():
        score = estimate_risk_score(bot.pos, ignore_robots)
        main.debug_drawer().draw_text("Risk: " + str(int(score * 100)),
                                      bot.pos, constants.Colors.White,
                                      "Defense")
        their_risk_scores.extend([score])

    # Sorts bot array based on their score
    zipped_array = zip(their_risk_scores, main.their_robots())
    sorted_array = sorted(zipped_array, reverse=True)
    sorted_bot = [bot for (scores, bot) in sorted_array]

    area_def_pos = create_area_defense_zones(ignore_robots)

    return area_def_pos, sorted_bot[0], sorted_bot[1]
