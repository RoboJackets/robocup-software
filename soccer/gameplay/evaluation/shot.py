import constants
import evaluation.window_evaluator
import robocup
import math

## DEPRECATED: This functionality is now included in the C++ WindowEvaluator
## Evaluate the chance of a shot succeeding
#
# @param windowing_excludes - A list of robots to exclude from the window evaluator
# @param pos The Point where the shot will be taken from (the ball's location)
# @param target A Segment object specifying what we're trying to shoot at
# @param windowing_excludes A list of robots that should not be considered obstacles to this shot
# @param hypothetical_robot_locations A list of Points that we'll place robot obstacles at for this shot calculation
# @param debug If True, it draws some stuff on the field - TODO: which stuff?
# @return a tuple (chance of shot success, best window), where chance of shot success is a value between zero and one
def eval_shot(pos, target=constants.Field.TheirGoalSegment, windowing_excludes=[], hypothetical_robot_locations=[], debug=False):
    win_eval = evaluation.window_evaluator.WindowEvaluator()
    win_eval.excluded_robots = windowing_excludes
    win_eval.hypothetical_robot_locations = hypothetical_robot_locations
    windows, best = win_eval.eval_pt_to_seg(pos, target)

    if best != None:
        shot_vector = best.segment.center() - pos
        shot_dist = shot_vector.mag()

        # get the angle between the shot vector and the target segment, then normalize and positivize it
        angle_between_shot_and_window = abs(shot_vector.angle() - best.segment.delta().angle())
        while abs(angle_between_shot_and_window) > math.pi:
            angle_between_shot_and_window -= math.pi
        angle_between_shot_and_window = abs(angle_between_shot_and_window)


        # we don't care about the segment length, we care about the width of the corresponding segment perpendicular to the shot line
        perp_seg_length = abs(math.sin(angle_between_shot_and_window)) * best.segment.length()

        # the 'width' of the shot in radians
        angle = abs(math.atan2(perp_seg_length, shot_dist))

        # the wider available angle the shot has, the more likely it will make it
        # the farther the shot has to travel, the more likely that defenders can block it in time
        ShotAngleBaseline = (math.pi / 20.0)    # note: this angle choice is fairly arbitrary - feel free to tune it
        angle_score = min(angle / ShotAngleBaseline, 1.0)
        longest_possible_shot = math.sqrt(constants.Field.Length**2 + constants.Field.Width)
        dist_score = 1.0 - (shot_dist / longest_possible_shot)
        shot_chance = 0.7*angle_score + 0.3*dist_score  # note: the weights are fairly arbitrary and can be tuned


        if debug:
            # raise NotImplementedError("Draw the shot chance on the line")
            # raise NotImplementedError("draw the shot line and window")
            pass

        return shot_chance, best
    else:
        return 0.0, None
