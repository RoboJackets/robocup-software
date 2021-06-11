import sys
sys.path.insert(1, "../../stp")
import rc
import numpy as np
from ball import Ball

class DefensivePosition:

    robot = rc.Robot.generate_test_robot(1)

    ## Predicts the impending kick direction based on the orientation of the robot and the
    #  angle of approach
    #
    # @param robot: The robot which we want to estimate
    # @return Angle of most likely kick
    def predict_kick_direction(robot):
        pose = robot.pose()
        twist = robot.twist()
        angle = pose[2]
        pos = pose[:-1]
        angle_vel = twist[2]
        
        # Use distance from bot to ball to predict time it takes to intercept
        # Calculates direct robot to future ball position
        inst_ball_time = Ball.time_to_ball(robot)
        
        future_ball_pos = Ball.predict_pos(inst_ball_time)
        change = future_ball_pos - ball.pos
        direction = change/np.linalg.norm(change) #direction the ball travels in
        zero_angle = [0, 1]
        #find angle of approach
        dot_product = np.dot(zero_angle, direction)
        ball_angle_predict = np.arccos(dot_product)
        
        # Predict the robot direction based on angular velocity and angle
        robot_angle_predict = angle + angle_vel * inst_ball_time

        filter_coeff = 0.7
        return filter_coeff * robot_angle_predict + (1 - filter_coeff) * ball_angle_predict

    def get_points_from_rect(field, step=0.5):
        out = []
        goal_width = field.penalty_long_dist_m / 2
        goal_height = field.penalty_short_dist_m
        x = -(field.width_m / 2)
        max_x = (field.width_m / 2)
        y = 0
        max_y = field.length_m
        
        while x <= max_x:
            while y <= max_y:
                if ball.pos[1] > height or abs(ball.pos[0]) > width:
                    out.append([x, y])
                y += step
            x += step
        
        return out

    def create_area_defense_zones(field, ball, ignore_robots):
        # Create a 2D list [N][M] where N is the bucket
        # and M is the index along that point
        # The lists contains (robocup.Point, score)
        points = [[]]
        
        # Amnt each bucket holds
        angle_inc = math.pi / 10
        # Amnt to inc each value by
        dist_inc = 2.5
        
        # Holds the float angle (Radians)
        angle = 0.0
        # Holds the integer bucket based on angle
        angle_cnt = 0
        # Holds dist
        dist = dist_inc

        score_sum = 0.0
        point_cnt = 0
        
        x = -(field.width_m / 2)
        max_x = (field.width_m / 2)
        y = 0
        max_y = field.length_m
        
        # Populates all the buckets with values
        while angle < 2 * math.pi:

            point = ball.pos
            while (point[0] >= x and point[0] <= max_x and point[1] >= y and point[1] <= max_y):
                x = math.cos(angle) * dist + ball.pos[0]
                y = math.sin(angle) * dist + ball.pos[1]
                point = [x, y]
                score = estimate_risk_score(ball, point, ignore_robots)

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
        bucket_pt_sum = [0, 0]
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

    '''
    ## Estimates how dangerous an enemy robot can be at a certain point
#  Takes pass / shot and time to execute on ball into account
#
# @param pos: Position in which to estimate score at
# @return Risk score at that point
def estimate_risk_score(pos: robocup.Point,
                        ignore_robots: List[robocup.Robot] = []) -> float:
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
            weights[5] * (1 - dist / max_dist) ** 2

    return score / sum(weights)
    '''
    
    ## Estimates how dangerous an enemy robot can be at a certain point
    #  Takes pass / shot and time to execute on ball into account
    #
    # @param pos: Position in which to estimate score at
    # @return Risk score at that point
    def estimate_risk_score(ball, pos, ignore_robots):
        # Caches some kick eval functions
        max_time = 1
        max_ball_vel = 8  # m/s per the rules
        est_turn_vel = 8  # rad/s per a random dice roll (Over estimates oppnents abilities)
        
        robotRadius = 0.09 #radius of robot
        #_, pass_score = kick_eval.eval_pt_to_robot(ball.pos, pos) - 54 lines to return 
        # origin = ball.pos
        # target = pos
        # target_width = 2 * robotRadius
        dir_raw = pos - ball.pos
        dir_un = [-dir_raw[1], dir_raw[0]]
        dir_n = sqrt(dir_un[0]**2 + dir_un[1]**2)
        dir = dir_un / dir_n
        seg = [pos + dir * (robotRadius), pos - dir * (robotRadius)]
        
        center = [(seg[0][0] + seg[1][0]) / 2, (seg[0][1] + seg[1][1]) / 2]
        '''
        return eval_pt_to_seg(origin, seg);
    float target_width = get_target_angle(origin, target);
    Point left = seg.pt[0] - ball.pos;
    Point right = seg.pt[1] - ball.pos;

    return static_cast<float>(abs(left.angle_between(right)));

    // Polar bot locations
    // <Dist, Angle>
    vector<tuple<float, float> > bot_locations = convert_robots_to_polar(origin, center);

    // Convert polar to mean / std_dev / Vertical Scales
    vector<float> bot_means;
    vector<float> bot_st_devs;
    vector<float> bot_vert_scales;

    bot_means.reserve(bot_locations.size());
    bot_st_devs.reserve(bot_locations.size());
    bot_vert_scales.reserve(bot_locations.size());

    float dist_past_target{};

    for (tuple<float, float>& loc : bot_locations) {
        bot_means.push_back(get<1>(loc));
        // Want std_dev in radians, not XY distance
        bot_st_devs.push_back(std::atan(static_cast<float>(kick_evaluator::PARAM_robot_std_dev) / get<0>(loc)));

        // Robot Past Target
        dist_past_target = static_cast<float>(get<0>(loc) - (origin - center).mag());

        // If robot is past target, only use the chance at the target segment
        if (dist_past_target > 0 && fabs(get<1>(loc)) < M_PI / 2) {
            // Evaluate a normal distribution at dist away and scale
            bot_vert_scales.push_back(1 - erf(dist_past_target / (static_cast<float>(kick_evaluator::PARAM_robot_std_dev) * sqrt(2))));
        } else {
            bot_vert_scales.push_back(1);
        }
    }

    // Create function with only 1 input
    // Rest are bound to constant values
    function<tuple<float, float>(float)> ke_func =
        [&](float x){
            return eval_calculation(x, 
                static_cast<float>(kick_evaluator::PARAM_kick_mean),
                static_cast<float>(kick_evaluator::PARAM_kick_std_dev),
                std::cref(bot_means),
                std::cref(bot_st_devs),
                std::cref(bot_vert_scales),
                target_width / -2,
                target_width / 2);
        };
        

    // No opponent robots on the field
    if (bot_means.empty()) {
        // Push it off to the side
        bot_means.push_back(4);
        // Must be non-zero as 1 / bot_st_dev is used
        bot_st_devs.push_back(0.1);
        bot_vert_scales.push_back(0.001);

        // Center will always be the best target X with no robots
        return pair<Point, float>(center, get<0>(ke_func(0)));
    }

    ParallelGradient1DConfig parallel_config;
    KickEvaluator::init_gradient_configs(parallel_config, ke_func, bot_means, bot_st_devs,
                                         target_width / -2, target_width / 2);

    // Create Gradient Ascent Optimizer and run it
    ParallelGradientAscent1D optimizer(&parallel_config);

    optimizer.execute();

    // Grab the lcoal max values and their X location
    vector<float> max_x_values = optimizer.get_max_x_values();
    vector<float> max_values = optimizer.get_max_values();

    // Default to a local max
    int index = distance(max_values.begin(), max_element(max_values.begin(), max_values.end()));
    float max_x = max_x_values.at(index);
    float max_chance = max_values.at(index);

    // See if there is a segment which is longer
    // Since local maxes stop on either side of the segment
    if (max_x_values.size() > 1) {
        for (int i = 0; i < max_x_values.size() - 1; i++) {
            // Finds the score at the average between two local maxes
            float mid_point = (max_x_values.at(i) + max_x_values.at(i + 1)) / 2;
            float chance = get<0>(ke_func(mid_point));

            // chance >= max_chance
            if (chance > max_chance || nearly_equal(chance, max_chance)) {
                max_x = mid_point;
                max_chance = chance;
            }
        }
    }

    // Angle in reference to the field
    float real_max_angle = static_cast<float>(max_x + (center - origin).angle());
    Line best_kick_line(origin, origin + Point{cos(real_max_angle), sin(real_max_angle)});

    // Return point on target segment and chance
    return pair<Point, float>(target.nearest_point(best_kick_line), max_chance);
        '''
        return 0.0

    '''
    ## Decides where the best positions for defense is
#
# @return area_defense_position, highest_risk_robot, 2nd_highest_risk_robot
def find_defense_positions(
    ignore_robots: List[robocup.Robot] = []
) -> Tuple[robocup.Point, robocup.OpponentRobot, robocup.OpponentRobot]:

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
    '''

    def find_defense_positions(ignore_robots = []):
        return None, None, None

    '''
    ## Finds the line segment between the mark_pos and the highest danger shot point
# Cuts off the portion of the line that is inside of the goal box
#
# @param mark_pos: Point (usually robot position) to defend against
# @param robot: The robot assigned to defend
# @param ball: Is the marked position the ball? Defaults to robot - used for offsets
# @param kick_eval: kick evaluator, not required, but fewer steps if it's included here vs. recreated
# @return Tuple: LineSegment to defend on , shot_pt from kick_eval
def goalside_mark_segment(
    mark_pos: robocup.Point,
    robot: robocup.OurRobot,
    ball: bool = False,
    kick_eval: Optional[robocup.KickEvaluator] = None
) -> Tuple[robocup.Segment, robocup.Point]:
    if kick_eval is None:
        kick_eval = robocup.KickEvaluator(main.system_state())

    # Define the segments where the defender can go closest the goal
    offset = constants.Robot.Radius
    goal_rect_padded = constants.Field.OurGoalZoneShapePadded(offset)

    # Find best shot point from threat
    kick_eval.add_excluded_robot(robot)
    shot_pt, shot_score = kick_eval.eval_pt_to_our_goal(mark_pos)
    kick_eval.excluded_robots.clear()

    # End the mark line segment 1 radius away from the opposing robot
    # Or 1 ball radius away if marking a position
    if not ball:
        adjusted_mark_pos = mark_pos - (
            mark_pos - shot_pt).normalized() * 2 * constants.Robot.Radius
    else:
        adjusted_mark_pos = mark_pos - (mark_pos - shot_pt
                                        ).normalized() * constants.Ball.Radius

    shot_seg = robocup.Segment(adjusted_mark_pos, shot_pt)
    tmp = goal_rect_padded.segment_intersection(shot_seg)

    if tmp is None:
        return shot_seg, shot_pt

    intersections = sorted(tmp, key=lambda pt: pt.y, reverse=True)

    # Correction for when there is no segment because the opposing robot is inside a radius of our goalzone
    if len(intersections) == 0:
        intersections.append(adjusted_mark_pos)

    return robocup.Segment(adjusted_mark_pos, intersections[0]), shot_pt
    '''

    def goalside_mark_segment(mark_pos, robot, ball = False, kick_eval = None):
        return None, None


## Finds the line segment between the mark_pos and the ball
# @param mark_pos: Point (usually robot position) to defend against
# @return: LineSegment to defend on
    def ballside_mark_segment(
        mark_pos: robocup.Point,
        ball_pos: Optional[robocup.Point] = None) -> robocup.Line:
        # offsets on ball and mark_pos sides
        if ball_pos is None:
            ball_pos = main.ball().pos
        offsets = [constants.Robot.Radius, constants.Ball.Radius]
        mark_line_dir = (ball_pos - mark_pos).normalized()
        ball_mark_line = robocup.Segment(
            ball_pos - mark_line_dir * constants.Ball.Radius,
            mark_pos + mark_line_dir * 2.0 * constants.Robot.Radius)
        # End the mark line segment 1 radius away from the opposing robot
        # Or 1 ball radius away if marking a position
        return ball_mark_line

    '''
