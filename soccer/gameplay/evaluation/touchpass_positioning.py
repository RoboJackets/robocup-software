import robocup
import constants
import main
import evaluation.passing

## The Touchpass positioning file finds the best location within a rectangle to ricochet
# a ball into the goal.
#
# By default, this will select a rectangle that is across the field from the current ball position.
# The best location is found by multiplying the chance the pass from the ball's position to the point will make it
# with the chance a shot into a goal will make it. The greatest probability is selected, and returned.
#
# Example usage:
# tpass = evaluation.touchpass_positioning
# tpass.eval_best_receive_point(main.ball().pos, None, pass_bhvr.get_robots())


## Returns a robocup.Rect object that is the default location to be evaluated
# This rectangle will only include points with a lower y value that the ball's current location, and will be on the side of the field
# opposite to the ball.
#
# Takes a current ball position/initial kick position (robocup.Point)
def generate_default_rectangle(kick_point):
    offset_from_edge = 0.25
    offset_from_ball = 0.4
    # offset_from_ball = 0.7

    if kick_point.x > 0:
        # Ball is on right side of field
        toReturn = robocup.Rect(robocup.Point(0, min(constants.Field.Length - offset_from_edge, main.ball().pos.y - offset_from_ball)),
                robocup.Point(-constants.Field.Width / 2 + offset_from_edge, min(constants.Field.Length * 3 / 4, main.ball().pos.y - 2)))
    else:
        # Ball is on left side of field
        toReturn = robocup.Rect(robocup.Point(0, min(constants.Field.Length - offset_from_edge, main.ball().pos.y - offset_from_ball)),
                robocup.Point(constants.Field.Width / 2 - offset_from_edge, min(constants.Field.Length * 3 / 4, main.ball().pos.y - 2)))
    return toReturn

## Returns a list of robocup.Segment object that represent candidate lines. Takes in a robocup.Rect.
#
# These lines will be evaluated later by the window_evaluator.
def get_segments_from_rect(rect, threshold=0.75):
    outlist = []
    currentx = rect.min_x()
    currenty = rect.max_y()

    # Loop through from top left to bottom right

    while currentx <= rect.max_x():
        currenty = rect.max_y()
        # Don't include goal area.
        if constants.Field.TheirGoalShape.contains_point(robocup.Point(currentx, rect.min_y())):
            continue
        while constants.Field.TheirGoalShape.contains_point(robocup.Point(currentx, currenty)):
            currenty = currenty - threshold

        candiate = robocup.Segment(robocup.Point(currentx, rect.min_y()), robocup.Point(currentx, currenty))
        outlist.extend([candiate])
        currentx = currentx + threshold
    currentx = rect.min_x()
    return outlist

## Evaluates a single point, and returns the probability of it making it.
#
# The value returned is the probability that a pass from the kick_point to the receive_point will make it,
# multiplied by the probability that a goal can be scored from receive_point. This probablity will be between 0 and 1.
def eval_single_point(kick_point, receive_point, ignore_robots=[]):
    if kick_point is None:
        if main.ball().valid:
            kick_point = main.ball().pos
        else:
            return None

    currentChance = evaluation.passing.eval_pass(kick_point, receive_point, ignore_robots)
    # TODO dont only aim for center of goal. Waiting on window_evaluator returning a probability.
    targetPoint = constants.Field.TheirGoalSegment.center()
    currentChance = currentChance * evaluation.passing.eval_pass(receive_point, targetPoint, ignore_robots)
    return currentChance


## Finds the best receive point for a bounce-pass.
#
# Takes in an initial kick point and an optional evaluation zone.
def eval_best_receive_point(kick_point, evaluation_zone=None, ignore_robots=[]):
    win_eval = robocup.WindowEvaluator(main.system_state())
    for r in ignore_robots:
        win_eval.add_excluded_robot(r)

    targetSeg = constants.Field.TheirGoalSegment
    # Autogenerate kick point
    if evaluation_zone == None:
        evaluation_zone = generate_default_rectangle(kick_point)

    segments = get_segments_from_rect(evaluation_zone)

    if segments == None or len(segments) == 0:
        # We can't do anything.
        return None

    bestChance = None

    for segment in segments:
        main.system_state().draw_line(segment, constants.Colors.Blue, "Candidate Lines")
        _, best = win_eval.eval_pt_to_seg(kick_point, segment)
        if best == None: continue

        currentChance = best.shot_success
        # TODO dont only aim for center of goal. Waiting on window_evaluator returning a probability.
        receivePt = best.segment.center()

        _, best = win_eval.eval_pt_to_seg(receivePt, targetSeg)
        if best == None: continue

        currentChance = currentChance * best.shot_success
        if bestChance == None or currentChance > bestChance:
            bestChance = currentChance
            targetPoint = best.segment.center()
            bestpt = receivePt

    if bestpt == None:
        return None

    return bestpt, targetPoint, bestChance
