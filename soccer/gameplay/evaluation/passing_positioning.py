import constants
import math
import robocup
import main
import evaluation.field
import evaluation.passing
import evaluation.shooting
import functools

## Finds the best location with a rectangle to pass the ball into
#
# By default, this is use the half of the field in front of the ball (Or the opponents half)
# The best location is found by combining the pass chance by the 
# openness, field position coefficients, and shot chance
#
# Example usage:
# point, score = evaluation.passing.eval_best_receive_point(main.ball().pos)


## Evaluates a single point and returns the overall coefficient for the area
#
# @param kick_point: Point where we are kicking from
# @param ignore_robots: Robots to ignore
# @param min_pass_dist: Minimum distance that we should ever pass
# @param field_weights: A tuple of the 3 difference weights to apply to field position 
#               (Centerness, Distance to their goal, Angle off their goal)
# @param weights: A tuple of the 4 different weights to apply to the evaulations overall
#               (space, field_position, shot_chance, kick_proximty)
# @param recieve_x: X position of the receive point
# @param recieve_y: Y position of the receive point
# @return Returns a score between 0 and 1 on how good of pass would be
def eval_single_point(kick_point, ignore_robots, min_pass_dist, field_weights,
                      weights, receive_x, receive_y):

    receive_point = robocup.Point(receive_x, receive_y)

    if kick_point is None:
        if main.ball().valid:
            kick_point = main.ball().pos
        else:
            return 0

    w = constants.Field.Width
    l = constants.Field.Length
    x_offset = .1 * w
    y_offset = .1 * l

    # Check boundaries
    # Can be smoothed for a better solution
    robot_offset = constants.Robot.Radius * 6
    if (receive_point.x - x_offset < w / -2 or
            receive_point.x + x_offset > w / 2 or
            receive_point.y - y_offset < 0 or
            receive_point.y + y_offset > constants.Field.Length or
            constants.Field.TheirGoalZoneShape.contains_point(
                receive_point + robocup.Point(0, y_offset)
                + robocup.Point(robot_offset, robot_offset)) or
            constants.Field.TheirGoalZoneShape.contains_point(
                receive_point + robocup.Point(0, y_offset)
                - robocup.Point(robot_offset, robot_offset))):
        return 0

    # Check if we are too close to the ball
    if ((receive_point - kick_point).mag() < min_pass_dist):
        return 0

    shotChance = evaluation.shooting.eval_shot(receive_point,
                                                   ignore_robots)

    passChance = evaluation.passing.eval_pass(kick_point, receive_point,
                                              ignore_robots)

    space = evaluation.field.space_coeff_at_pos(receive_point, ignore_robots)
    fieldPos = evaluation.field.field_pos_coeff_at_pos(
        receive_point, field_weights[0], field_weights[1], field_weights[2])
    distance = math.exp(-1 * (kick_point - receive_point).mag())

    # All of the other scores are based on whether the pass will actually make it to it
    # Not worth returning a great position if we cant even get a pass there
    totalChance = passChance * (
        weights[0] * (1 - space) + weights[1] * fieldPos + weights[2] *
        shotChance + weights[3] * (1 - distance))

    return totalChance / math.fsum(weights)


## Finds the best position to pass to
#
# @param kick_point: Point that we are passing from
# @param ignore_robots: Robots to ignore when calculating scores
# @param field_weights: A tuple of the 3 difference weights to apply to field position 
#               (Centerness, Distance to their goal, Angle off their goal)
# @param nelder_mead_args: A tuple of the nelder mead optimization args
#               (Starting point, Starting step, Exit condition, Reflection, Expansion,
#                Coontraction, Shrink, Max iterations, Max function value, Max value exit condition)
# @param weights: A tuple of the 4 different weights to apply to the evaulations overall (Weights are normalized)
#               (space, field_position, shot_chance, kick_proximity)
# @return bestPoint and bestScore in that order
def eval_best_receive_point(kick_point,
                            ignore_robots=[],
                            min_pass_dist=0.0,
                            field_weights=(0.1, 3.2, 0.1),
                            nelder_mead_args=(robocup.Point(0.5, .5),
                                              robocup.Point(0.01, 0.01), 1, 2,
                                              0.75, 0.5, 50, 1, 0.1),
                            weights=(1, 4, 15, 1)):
    pythfunc = functools.partial(eval_single_point, kick_point, ignore_robots,
                                 min_pass_dist, field_weights, weights)
    cppfunc = robocup.PythonFunctionWrapper(pythfunc)
    nmConfig = robocup.NelderMead2DConfig(
        cppfunc, kick_point, nelder_mead_args[0], nelder_mead_args[1],
        nelder_mead_args[2], nelder_mead_args[3], nelder_mead_args[4],
        nelder_mead_args[5], nelder_mead_args[6], nelder_mead_args[7],
        nelder_mead_args[8])
    nm = robocup.NelderMead2D(nmConfig)
    nm.execute()

    return nm.getPoint(), nm.getValue()
