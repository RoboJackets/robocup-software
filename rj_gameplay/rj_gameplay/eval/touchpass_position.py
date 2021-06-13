import sys
sys.path.insert(1, "../../stp")
import rc
import Pass
import chip
from typing import List, Optional, Tuple, Union

class TouchpassPosition:
    
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
    
    ## Returns a rectangle (two diagonal corners) that is the default location to be evaluated
    # This rectangle will only include points with a lower y value that the ball's current location, and will be on the side of the field
    # opposite to the ball.
    #
    # @param point current ball position/initial kick position (robocup.Point)
    # @param field rc.Field
    # @param ball rc.Ball
    def generate_default_rectangle(point, field, ball):
        edge_offset = 0.25
        ball_offset = 0.4
        
        if point[0] > 0:
            #ball on right side of field
            from_point = [0, min(field.length_m - edge_offset, ball.pos[1] - ball_offset)]
            to_point = [((-field.width_m / 2) + edge_offset), min(field.length_m * 3 / 4, ball.pos[1] - 2)]
            rectangle = [from_point, to_point]
            return rectangle
        else: 
            #left side of field
            from_point = [0, min(field.length_m - edge_offset, ball.pos[1] - ball_offset)]
            to_point = [((field.width_m / 2) - edge_offset), min(field.length_m * 3 / 4, ball.pos[1] - 2)]
            rectangle = [from_point, to_point]
            return rectangle
        
    ## Returns a list of robocup.Segment object that represent candidate lines. Takes in a rectangle (two diagonal corner points)
    #
    # These lines will be evaluated later by the window_evaluator.
    def get_segments_from_rect(rect, threshold):
        outlist = []
        curr_x = min(rect[0][0], rect[1][0]) #current x
        max_x = max(rect[0][0], rect[1][0])
        max_y = max(rect[0][1], rect[1][1])
        min_y = min(rect[0][1], rect[1][1])
        
        # loop through top left to bottom right
        while curr_x <= max_x:
            curry = max_y # you get variable names like this when you have an Indian writing your code. Current y 
            # don't include goal area
            width = field.penalty_long_dist_m / 2
            height = (field.length - field.penalty_short_dist_m)
            while (curry <= height and abs(curr_x) > width):
                curry -= threshold
            candidate = [[curr_x, min_y], [curr_x, curry]]
            outlist.append(candidate)
            curr_x += threshold
        curr_x = min_x
        return outlist

    ## Evaluates a single point, and returns the probability of it making it.
    #
    # The value returned is the probability that a pass from the kick_point to the receive_point will make it,
    # multiplied by the probability that a goal can be scored from receive_point. This probablity will be between 0 and 1.
    def eval_single_point(ball, field, kick_point, receive_point, targetPoint, ignore_robots):
        if kick_point is None:
            if ball.valid:
                kick_point = ball.pos
            else:
                return None

        currentChance = Pass.eval_pass(kick_point, receive_point,
                                                 ignore_robots)
        # TODO don't only aim for center of goal. Waiting on window_evaluator returning a probability.
        if targetPoint is None:
            targetPoint = [0,field.length_m]
        currentChance = currentChance * Pass.eval_pass(receive_point, targetPoint, ignore_robots)
        return currentChance
        
    ## Finds the best receive point for a bounce-pass.
    # @param kick_point the point we will kick from (robocup.Point)
    # @param evaluation_zone the zone that will be evaluated to try to find points.
    # If none, it will try to guess a good receive area.
    # This is a robocup.Rect
    # @param ignore_robots a list of robots to be ignored when trying to find the best receive point.
    def eval_best_receive_point(kick_point, evaluation_zone, ignore_robots):
        #TODO: finish this function
        return None, None, None

        # Autogenerate kick point
        if evaluation_zone is None:
            evaluation_zone = generate_default_rectangle(kick_point)

        segments = get_segments_from_rect(evaluation_zone)

        if segments is None or len(segments) == 0:
            # We can't do anything.
            return None, None, None
        bestChance = None

        for segment in segments:
            main.debug_drawer().draw_line(segment, constants.Colors.Blue,
                                      "Candidate Lines")
        _, best = win_eval.eval_pt_to_seg(kick_point, segment)

        if best is None: continue

        currentChance = best.shot_success
        # TODO don't only aim for center of goal. Waiting on window_evaluator returning a probability.
        receivePt = best.segment.center()

        _, best = win_eval.eval_pt_to_seg(receivePt, targetSeg)

        if best is None: continue

        currentChance = currentChance * best.shot_success
        if bestChance is None or currentChance > bestChance:
            bestChance = currentChance
            targetPoint = best.segment.center()
            bestpt = receivePt

    if bestpt is None:
        return None, None, None

    return bestpt, targetPoint, bestChance
