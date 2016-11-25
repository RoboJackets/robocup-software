import constants
import math
import robocup
import main
import evaluation.field

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
    receive_seg = robocup.Segment(to_point + pass_perp * receive_seg_half_len,
                                  to_point + pass_perp * -receive_seg_half_len)

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


## Finds the best location wiht a rectangle to pass the ball into
#
# By default, this is use the half of the field in front of the ball (Or the opponents half)
# The best location is found by combining the pass chance by the 
# openness and field position coefficients
#
# Example usage:
# evaluation.passing.eval_best_receive_point(main.ball().pos)
#
# Which finds the best pass from the ball position

## Returns a robocup.Rect object that is the default location
#
# @param pos: Passing position
# @return Returns a rectangle of half the field in front of pos
#  or their half of the field if pos is in their half
def generate_default_rectangle(pos):
    w = constants.Field.Width
    l = constants.Field.Length
    offset_from_edge = w*0.1

    bot_left  = robocup.Point(-1*w/2 + offset_from_edge, min(l/2, pos.y))
    top_right = robocup.Point(w/2 - offset_from_edge, min(l-offset_from_edge, bot_left.y + l/2))

    return robocup.Rect(bot_left, top_right)


## Returns a list of robocup.Point objects that represent candidate recieve points
#
# @param rect: Rectangle to search through
# @param pos: Position to pass from
# Returns a list of points in the rectangle to test
def get_points_from_rect(rect, pos, threshold=0.75):
    outlist = []
    currentx = rect.min_x()
    currenty = rect.min_y()

    # Loop through from bottom left to top right, row by row
    while currenty <= rect.max_y():
        while currentx <= rect.max_x():
            # If within the goalie area
            if constants.Field.TheirGoalZoneShape.contains_point(robocup.Point(currentx, currenty)):
                currentx += threshold
                continue

            candidate = robocup.Point(currentx, currenty)

            if ( (candidate - pos).mag() > threshold ):
                outlist.extend([candidate])
            currentx += threshold
    
        currenty += threshold
        currentx = rect.min_x()

    return outlist

## Evaluates a single point and returns the overall coefficient for the area
#
# @return Returns a score between 0 and 1 on how good of pass it is
def eval_singl_point(kick_point,
                     receive_point,
                     ignore_robots=[]):
    
    if kick_point is None:
        if main.ball().valid:
            kick_point = main.ball().pos
        else:
            return None

    # TODO: Figure out shot chance and add it in
    shotChance = 1
    passChance = eval_pass(kick_point, 
                                              receive_point,
                                              ignore_robots)
    space    = evaluation.field.space_coeff_at_pos(receive_point, ignore_robots)
    fieldPos = evaluation.field.field_pos_coeff_at_pos(receive_point)

    # TODO: Make this more advanced
    # Make sure a robot can get to the position (Distancce from our closet robot?)
 
    # All of the other scores are based on whether the pass will actually make it to it
    # Not worth returning a great position if we can even get a pass there
    spaceCoeff = 1
    fieldCoeff = 8
    shotCoeff  = 10
    totalChance = passChance * ( spaceCoeff*(1-space) + fieldCoeff*fieldPos + shotCoeff*shotChance)
    totalCahnce /= (spaceCoeff + fieldCoeff + shotCoeff)
    return totalChance

## Finds the best position to pass to
#
# @param kick_point: Point that we are passing from
# @param evaluation_zone: Area to evaluate passes to
# @param ignore_robots: Robots to ignore when calculating scores
# @param debug: Displays pass lines and scores onto the field when true
# @return bestPoint and bestScore in that order
def eval_best_receive_point(kick_point,
                            evaluation_zone=None,
                            ignore_robots=[], 
                            debug=False):
    win_eval = robocup.WindowEvaluator(main.system_state())
    for r in ignore_robots:
        win_eval.add_excluded_robot(r)

    if evaluation_zone is None:
        evaluation_zone = generate_default_rectangle(kick_point)

    points = get_points_from_rect(evaluation_zone, kick_point, 0.5)

    if points is None or len(points) == 0:
        # Nothing can be done
        return None

    bestScore = None
    bestPointt = None

    # Finds best score out of all the points
    for currentPoint in points:
        currentScore = eval_singl_point(kick_point, currentPoint, ignore_robots)

        if (debug):
            score_color = (round(currentScore*255), 0, round((1-currentScore)*255))
            main.system_state().draw_line(robocup.Segment(kick_point, currentPoint), score_color, "Debug")
        
        

        if bestScore is None or currentScore > bestScore:
            bestScore = currentScore
            bestPoint = currentPoint

    # Returns None if we can't find anythign
    if bestPoint is None:
        return None

    main.system_state().draw_line(robocup.Segment(kick_point, bestPoint), constants.Colors.Red, "Debug")

    return bestPoint, bestScore