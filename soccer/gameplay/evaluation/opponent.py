import main
import robocup
import constants


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
# @param direction_weight: How much to weight the positive y direction
#     If < 1, then robots < pos.y are weighted by direction_weight
def get_closest_opponent(pos, direction_weight=1, excluded_robots=[]):

    closest_bot, closest_dist = None, float("inf")
    for bot in main.their_robots():
        if bot.visible:
            dist = (bot.pos - pos).mag()

            if (pos.y <= bot.pos.y):
                dist *= direction_weight
            else:
                dist *= (1 - direction_weight)

            if dist < closest_dist:
                closest_bot = bot
                closest_dist = dist

    return closest_bot
