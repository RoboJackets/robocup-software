import main
import robocup
import constants
import math


## Estimates the number of robots on offense on the opposing team
#
# @return Num of robots on offense
def num_on_offense():
    # Complementary filter based on...
    #	Distance to their goal
    #	Distance to the ball
    goal_loc = robocup.Point(0, constants.Field.Length)
    corner_loc = robocup.Point(constants.Field.Width / 2, 0)
    ball_loc = main.ball().pos

    max_goal_dis = (goal_loc - corner_loc).mag()
    ball_to_goal = (goal_loc - ball_loc).mag()
    offense_ctr = 0

    filter_coeff = 0.7
    score_cutoff = .3

    # For each of their robots
    for bot in main.their_robots():
        if bot.visible:
            dist_to_ball = (bot.pos - ball_loc).mag()
            dist_to_goal = (bot.pos - goal_loc).mag()

            goal_coeff = dist_to_goal / max_goal_dis
            if ball_to_goal != 0:
                ball_coeff = 1 - (dist_to_ball / ball_to_goal)
            else:
                ball_coeff = 1
            ball_coeff = max(0, ball_coeff * ball_coeff)

            score = filter_coeff * goal_coeff + (1 - filter_coeff) * ball_coeff

            # Only count if their score is above the cutoff
            if (score > score_cutoff):
                offense_ctr += 1

    return offense_ctr


## Returns the closest opponent to the pos inclusive of the directional weight
#
# @param direction_weight: How much to weight the positive y direction,
#     0 <= direction weight <= 2
#     If < 1, then robots < pos.y are weighted by direction_weight
def get_closest_opponent(pos, direction_weight=0, excluded_robots=[]):

    closest_bot, closest_dist = None, float("inf")
    for bot in main.their_robots():
        if bot.visible and bot not in excluded_robots:
            dist = (bot.pos - pos).mag()

            if (pos.y <= bot.pos.y):
                dist *= (2 - direction_weight / 2)
            else:
                dist *= (2 + direction_weight / 2)

            if dist < closest_dist:
                closest_bot = bot
                closest_dist = dist

    return closest_bot

## Gets list of threats
#
# @return sorted tuple of threat positions and score 
def get_threat_list(unused_threat_handlers):
    import evaluation.ball as ball_eval
    kick_eval = robocup.KickEvaluator(main.system_state())

    # List of (position, score, Robot/None)
    threats = []
    potential_threats = main.their_robots()

    # find the primary threat
    # if the ball is not moving OR it's moving towards our goal, it's the primary threat
    # if it's moving, but not towards our goal, the primary threat is the robot on their team most likely to catch it
    if (main.ball().vel.mag() > 0.4):
        if ball_eval.is_moving_towards_our_goal():
            # Add tuple of pos and score
            threats.append((main.ball().pos, 1, None))
        else:
            # Get all potential receivers
            potential_receivers = []
            for opp in potential_threats:
                if estimate_potential_recievers_score(opp) == 1:
                    potential_receivers.append((opp.pos, 1, opp))

            if len(potential_receivers) > 0:
                # Add best receiver to threats
                # TODO Calc shot chance
                best_tuple = min(potential_receivers,
                                    key=lambda rcrv_tuple: rcrv_tuple[1])
                threats.append((best_tuple[0], .81, best_tuple[2]))
            else:
                # Just deal with ball if no recievers
                threats.append((main.ball().pos, .9, None))
    else:
        # Assume opp is dribbling ball
        if not constants.Field.OurGoalZoneShape.contains_point(
                main.ball().pos):
            # TODO: Calc shot chance
            threats.append((main.ball().pos, 1, None))

    # if there are threats, check pass and shot chances
    # If the first item is not a ball, it is most likely a pass
    if len(threats) > 0 and threats[0][0] != main.ball().pos:
        for opp in potential_threats:

            # Exclude robots that have been assigned already
            excluded_bots = []
            for r in map(lambda bhvr: bhvr.robot, unused_threat_handlers):
                excluded_bots.append(r)

            threats.append((opp.pos, estimate_risk_score(
                opp, excluded_bots), opp))
    else:
        for opp in potential_threats:

            # Exclude all robots
            kick_eval.excluded_robots.clear()
            kick_eval.add_excluded_robot(opp)
            for r in main.our_robots():
                kick_eval.add_excluded_robot(r)

            point, shotChance = kick_eval.eval_pt_to_our_goal(opp.pos)

            # Note: 0.5 is a bullshit value
            threats.append((opp.pos, 0.5 * shotChance, opp))

    # Prevent threats from being below our goal line (causes incorrect pos)
    def _adjust_pt(threat):
        pt = threat[0]
        pt.y = max(pt.y, 0.1)
        return (pt,) + threat[1:]

    threats = list(map(_adjust_pt, threats))
    threats.sort(key=lambda threat: threat[1], reverse=True)

    return threats

## Estimate potential reciever score (likelihood of opponent passing to this robot)
#  
#  @param bot Robot to estimate score at
#  @return The potential receiver score at that point
def estimate_potential_recievers_score(bot):
    ball_travel_line = robocup.Line(main.ball().pos,
                                    main.ball().pos + main.ball().vel)

    dot_product = (bot.pos - main.ball().pos).dot(ball_travel_line.delta())
    nearest_pt = ball_travel_line.nearest_point(bot.pos)
    dx = (nearest_pt - main.ball().pos).mag()
    dy = (bot.pos - nearest_pt).mag()
    angle = abs(math.atan2(dy, dx))

    # Only returns 1 if the opp is moving in the opposite direction as the ball
    # and the angle between the ball ray starting at its current position and the opp position
    # is less than pi/4
    if (angle < math.pi / 4 and dot_product > 0):
        return 1
    else:
        return 0

## Estimate risk score based on old defense.py play
#  
#  @param bot Robot to estimate score at
#  @param exluded_Bots Robots to exclude from the defense when calculating shot
#  @return The risk score at that point (Shot chance * pass chance)
def estimate_risk_score(bot, excluded_bots=[]):
    import evaluation.passing as pass_eval
    kick_eval = robocup.KickEvaluator(main.system_state())

    excluded_bots.append(bot)

    passChance = pass_eval.eval_pass(
        main.ball().pos, bot.pos, excluded_robots=excluded_bots)

    kick_eval.excluded_robots.clear()
    for r in excluded_bots:
        kick_eval.add_excluded_robot(r)

    point, shotChance = kick_eval.eval_pt_to_our_goal(bot.pos)

    return passChance * shotChance

## Returns whether a position is marked by one of our robots
#
# @param pos Position to check
# @return True or False if a robot is on the mark line
def is_marked(pos):
    line = robocup.Line(pos, robocup.Point(0, 0))

    for bot in main.our_robots():
        if line.dist_to(bot.pos) < constants.Robot.Radius / 2:
            return True
    return False