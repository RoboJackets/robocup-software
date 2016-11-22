import main
import robocup
import constants

# Determines how much "space" there is at a pos
# Space is the emptiness of robots in a given area
# @returns Number > 0 representing the closeness of robots
# The higher the number, the more robots closer to the position
def space_coeff_at_pos(pos, excluded_robots=[]):
    max_dist = robocup.Point(pos.x - constants.Field.Width, pos.y - constants.Field.Length).mag()
    total = 0
    sensitivity = 4

    for bot in (main.their_robots()):# + main.our_robots()):
        if bot.visible and bot not in excluded_robots:
            u = sensitivity * (bot.pos - pos).mag() / max_dist;

            # Use Triweight kernal function (Force to be positive)
            total += max((35/32)*pow((1-pow(u,2)), 3), 0)

    return total



# Field Heuristic based on the position of the robot on the field
# Center is how close to the length-wise center of the field
# Dist is how close to the opponent goal
# Angle is the angle on the goal (0 is goal line, 90 is center line)
#def field_pos_coeff_at_pos(pos, center, dist, angl)
