import constants
import math
import robocup
import main
import evaluation.field
import evaluation.passing
import evaluation.shooting

## Finds the best location wiht a rectangle to pass the ball into
#
# By default, this is use the half of the field in front of the ball (Or the opponents half)
# The best location is found by combining the pass chance by the 
# openness, field position coefficients, and shot chance
#
# Example usage:
# point, score = evaluation.passing.eval_best_receive_point(main.ball().pos)

## Returns a robocup.Rect object that is the default location
#
# @param pos: Passing position
# @return Returns a rectangle of half the field in front of pos
#  or their half of the field if pos is in their half
def generate_default_rectangle(pos):
    w = constants.Field.Width
    l = constants.Field.Length
    offset_from_edge = w * 0.1

    bot_left  = robocup.Point(-1 * w / 2 + offset_from_edge, min( l / 2, pos.y ) )
    top_right = robocup.Point( w / 2 - offset_from_edge, min( l - offset_from_edge, bot_left.y + l / 2 ) )

    return robocup.Rect(bot_left, top_right)


## Returns a list of robocup.Point objects that represent candidate recieve points
#
# @param rect: Rectangle to search through
# @param pos: Position to pass from
# @param step: Distance between points
# @param min_dist: Minimum distance to check around our position
# @return: Returns a list of points in the rectangle to test
def get_points_from_rect(rect, pos, step=0.5, min_dist=1):
    outlist = []
    currentx = rect.min_x()
    currenty = rect.min_y()

    # Loop through from bottom left to top right, row by row
    while currenty <= rect.max_y():
        while currentx <= rect.max_x():

            # Check [X]% distance between this point and the goal line to remove "close" points to their goal zone
            goal_zone_thresh = (constants.Field.Length - currenty) * 0.25

            # If within the goalie area 
            if constants.Field.TheirGoalZoneShape.contains_point(robocup.Point(currentx, currenty + goal_zone_thresh)):
                currentx += step
                continue

            candidate = robocup.Point(currentx, currenty)

            # Force algorithm not to check within a min distance
            if ( (candidate - pos).mag() > min_dist ):
                outlist.extend([candidate])
            currentx += step
    
        currenty += step
        currentx = rect.min_x()

    return outlist

## Evaluates a single point and returns the overall coefficient for the area
#
# @param kick_point: Point where we are kicking from
# @param receive_point: Point to which are kicking to
# @param ignore_robots: Robots to ignore
# @param field_weights: A tuple of the 3 difference weights to apply to field position 
#               (Centerness, Distance to their goal, Angle off their goal)
# @param weights: A tuple of the 4 different weights to apply to the evaulations overall
#               (space, field_position, shot_chance, kick_proximty)
# @return Returns a score between 0 and 1 on how good of pass would be
def eval_singl_point(kick_point,
                     receive_point,
                     ignore_robots,
                     field_weights,
                     weights):
    
    if kick_point is None:
        if main.ball().valid:
            kick_point = main.ball().pos
        else:
            return None

    shotChance = 0

    # Dissallow shooting over midfield
    if (kick_point.y > constants.Field.Length / 2):
        # TODO: Replace with KickEval
        shotChance = evaluation.shooting.eval_shot(receive_point)
    
    # TODO: Replace with KickEval
    passChance = evaluation.passing.eval_pass(kick_point, receive_point)

    space    = evaluation.field.space_coeff_at_pos(receive_point, ignore_robots)
    fieldPos = evaluation.field.field_pos_coeff_at_pos(receive_point, field_weights[0], field_weights[1], field_weights[2])
    distance = math.exp( -1 * (kick_point - receive_point).mag() )

    # All of the other scores are based on whether the pass will actually make it to it
    # Not worth returning a great position if we cant even get a pass there
    totalChance = passChance * ( weights[0] * (1 - space) + \
                                 weights[1] * fieldPos + \
                                 weights[2] * shotChance + \
                                 weights[3] * (1 - distance) )
    
    return totalChance / math.fsum(weights)

## Finds the best position to pass to
#
# @param kick_point: Point that we are passing from
# @param evaluation_zone: Area to evaluate passes to
# @param ignore_robots: Robots to ignore when calculating scores
# @param field_weights: A tuple of the 3 difference weights to apply to field position 
#               (Centerness, Distance to their goal, Angle off their goal)
# @param weights: A tuple of the 4 different weights to apply to the evaulations overall (Weights are normalized)
#               (space, field_position, shot_chance, kick_proximity)
# @param debug: Displays pass lines and scores onto the field when true
# @return bestPoint and bestScore in that order
def eval_best_receive_point(kick_point,
                            evaluation_zone=None,
                            ignore_robots=[], 
                            field_weights=(0.1, 3.2, 0.1),
                            weights=(1, 4, 15, 1),
                            debug=False):

    if evaluation_zone is None:
        evaluation_zone = generate_default_rectangle(kick_point)

    points = get_points_from_rect(evaluation_zone, kick_point, 0.5, 0.5)

    if points is None or len(points) == 0:
        # Nothing can be done
        return None, 0

    # TODO: Setup to be a list of the top X points
    bestScore = None
    bestPointt = None

    # Finds best score out of all the points
    for currentPoint in points:
        currentScore = eval_singl_point(kick_point, currentPoint, ignore_robots, field_weights, weights)

        if (debug):
            score_color = (round(currentScore*255), 0, round((1-currentScore)*255))
            main.system_state().draw_line(robocup.Segment(kick_point, currentPoint), score_color, "Score")

        if bestScore is None or currentScore > bestScore:
            bestScore = currentScore
            bestPoint = currentPoint

    # Returns None if we can't find anythign
    if bestPoint is None:
        return None, 0

    if (debug):
        main.system_state().draw_line(robocup.Segment(kick_point, bestPoint), constants.Colors.Red, "Best Point")

    return bestPoint, bestScore