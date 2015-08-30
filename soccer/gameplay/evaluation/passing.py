import constants
import math
import robocup
import main


## Find the chance of a pass succeeding by looking at pass distance and what robots are in the way
# @param from_point The Point the pass is coming from
# @param to_point The Point the pass is being received at
# @param excluded_robots A list of robots that shouldn't be counted as obstacles to this shot
# @return a value from zero to one that estimates the probability of the pass succeeding
def eval_pass(from_point, to_point, excluded_robots=[]):
    # we make a pass triangle with the far corner at the ball and the opposing side touching the receiver's mouth
    # the side along the receiver's mouth is the 'receive_seg'
    # we then use the window evaluator on this scenario to see if the pass is open
    pass_angle = math.pi / 32.0
    pass_dist = to_point.dist_to(from_point)
    pass_dir = to_point - from_point
    pass_perp = pass_dir.perp_ccw()
    receive_seg_half_len = math.tan(pass_angle) * pass_dist
    receive_seg = robocup.Segment(to_point + pass_perp*receive_seg_half_len,
        to_point + pass_perp*-receive_seg_half_len)

    win_eval = robocup.WindowEvaluator(main.system_state())
    for r in excluded_robots:
        win_eval.add_excluded_robot(r)
    windows, best = win_eval.eval_pt_to_seg(from_point, receive_seg)

    # this is our estimate of the likelihood of the pass succeeding
    # value can range from zero to one
    # we square the ratio of best to total to make it weigh more - we could raise it to higher power if we wanted
    if best != None:
        return 0.8 * (best.segment.length() / receive_seg.length())**2
    else:
        # the pass is completely blocked
        return 0
