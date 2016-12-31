import constants
import math
import robocup
import main
import math

## Find the chance of a shot succeeding by looking at pass distance and what robots are in the way
#  The total goal angle as well as the  percent covered  is taken into account
# @param from_point The Point the shot is coming from
# @param excluded_robots A list of robots that shouldn't be counted as obstacles to this shot
# @return a value from zero to one that estimates the probability of the shot succeeding
def eval_shot(from_point, excluded_robots=[]):
    # Create a triangle between us and the two sides of the goal
    # Side along the goal is 'receive_seg'
    # The window evaluator is then used to see how open it is
    to_point = robocup.Point(0, constants.Field.Length)
    left_post = robocup.Point(-0.5*constants.Field.GoalWidth, constants.Field.Length)
    right_post = robocup.Point(0.5*constants.Field.GoalWidth, constants.Field.Length)

    left_vec = left_post - from_point
    right_vec = right_post - from_point

    pass_angle = math.atan2(left_vec.y, left_vec.x) - math.atan2(right_vec.y, right_vec.x)
    pass_angle = math.fabs(pass_angle)

    pass_dist = to_point.dist_to(from_point)
    pass_dir = to_point - from_point
    pass_perp = pass_dir.perp_ccw().normalized()
    receive_seg_half_len = math.tan(pass_angle/2) * pass_dist
    receive_seg = robocup.Segment(to_point + pass_perp * receive_seg_half_len,
                                  to_point + pass_perp * -receive_seg_half_len)

    win_eval = robocup.WindowEvaluator(main.system_state())
    for r in excluded_robots:
        win_eval.add_excluded_robot(r)
    windows, best = win_eval.eval_pt_to_seg(from_point, receive_seg)


    # TODO: We should also test a wider angle and check that against our best one
    # That way we can also get nearby robots

    # This is our estimate of the likelihood of the shot succeeding
    # Value can range from zero to one
    # Return 0 if shooting from our side of the field
    if best != None and (from_point.y > (constants.Field.Length / 2)):
        # The constants are chosen through graphing and selectively choosing a good weight for each
        # TODO: Use some more real life metrics to figure better constants
        # Standard deviation will relate directly with the total_percent
        # Standard deviation / shot power will relate to the dist_coeff

        # Percent of goal blocked by opponents
        seg_percent = math.pow(best.segment.length() / receive_seg.length(), 2) # ShotWidth / MaxShotWidth
        # Percent of goal visible to us (decreases as we move to the side)
        total_percent = math.pow(best.segment.length() / constants.Field.GoalWidth, 0.3) # ShotWidth / GoalWidth
        # Distance from the goal
        dist_coeff = math.pow(1 - (pass_dist / to_point.mag()), 0.8) # 1 - Shot / Max Shot
        
        return seg_percent * total_percent * dist_coeff
    else:
        # The shot is invalid
        return 0