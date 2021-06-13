import sys
import math
sys.path.insert(1, "../../stp")
import rc

class Field:
   ## Determines how much "space" there is at a pos
   #    "Space" is how empty an area of the field is
   # @param pos: point to evalute
   # @returns Number between 0 and 1 representing the closeness of robots
   # The higher the number, the more robots closer to the position
   def space_coeff_at_pos(point, excluded, their_robots, our_robots, field):
       if excluded is None:
        excluded = []
       if robots is None:
        robots = their_robots
        
       max_dist = sqrt(field.length_m**2 + (field.width_m**2)/4)
    
       total = 0
       sensitivity = 8
    
       for bot in robots:
            if bot.visible and bot not in excluded:
               pos = pose[:-1]
               dist = sqrt((pos[0] - point[0])**2 + (pos[1] - [point[1])**2)
               u = (sensitivity * dist) / max_dist

               # Use Triweight kernal function (Force to be positive)
               # Graph looks much like a normal distribution
               total += max((35 / 32) * pow((1 - pow(u, 2)), 3), 0)
                                                               
       return min(total, 1)
                                                               
    ## Field Heuristic based on the position of the robot on the field
    # Weight inputs are normalized
    #
    # @param pos: Position to evalute
    # @param center: How much to weight being close to the 'center of the field'
    # @param dist: How much to weight being close to the opponents goal
    # @param angl: How much to weight the angle between the robot and the goal (In turn, how small the goal is)
    # @param attacking_their_goal: Are we attacking their goal
    # @return Returns a number between 0 and 1 representing how good the position is
    def field_pos_coeff_at_pos(pos, field, center, dist, angle, attacking_their_goal):
        # Percent closeness to the center (Line between the two goals is the 'center line')
        centerValue = 1 - math.fabs(pos[0] / (field.width_m / 2))

    # Pencent closeness to their goal
    distValue = math.fabs(pos[1] / field.length_m)
    if (not attacking_their_goal):
        distValue = 1 - distValue

    # Angle of pos onto the goal, centerline is 0 degrees
    if (attacking_their_goal):
        angle = math.atan2(pos.x, field.length_m - pos[1])
        anglValue = 1 - math.fabs(angle / (math.pi / 2))
    else:
        anglValue = 1 - math.fabs(math.atan2(pos[0], pos[1]) / (math.pi / 2))

    # Normalize inputs
    total = center + dist + angl

    return (center * centerValue + dist * distValue + angl * anglValue) / total
