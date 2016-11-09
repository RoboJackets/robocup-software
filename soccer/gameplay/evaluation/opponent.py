import main
import robocup
import constants

def num_on_offense():
	# Complementary filter on...
	#	Closeness to their goal
	#	Closeness to the ball
	goal_loc   = robocup.Point(0, constants.Field.Length)
	corner_loc = robocup.Point(constants.Field.Width / 2, 0)
	ball_loc   = main.ball().pos
	max_goal_dis = (goal_loc - corner_loc).mag()
	ball_to_goal = (goal_loc - ball_loc).mag()
	offense_ctr = 0

	filter_coeff = 0.7
	cutoff = .3

	for bot in main.their_robots():
		if bot.visible:
			dist_to_ball = (bot.pos - ball_loc).mag()
			dist_to_goal = (bot.pos - goal_loc).mag()

			goal_coeff = dist_to_goal / max_goal_dis
			ball_coeff = 1 - (dist_to_ball / ball_to_goal)
			ball_coeff = max(0, ball_coeff*ball_coeff)

			coeff = filter_coeff * goal_coeff + (1 - filter_coeff) * ball_coeff

			if (coeff > cutoff):
				offense_ctr += 1

	return offense_ctr