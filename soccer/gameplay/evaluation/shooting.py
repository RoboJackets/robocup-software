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

    if from_point.y > (constants.Field.Length / 2):

        return chance
    else:
        # The shot is invalid
        return 0
