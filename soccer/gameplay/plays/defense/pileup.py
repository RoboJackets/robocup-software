import robocup
import constants
import play
import skills
import tactics
import main
import evaluation
import math
import random
'''
def fighting(r, x, y):    
    d = sqrt(abs(x - r.pos.x) + abs(y - r.pos.y))
    return d <= (3 * constants.Robot.Radius + constants.Ball.Radius)
'''


class Pileup(play.Play):
    def __init__(self):
        super().__init__(continuous=False)

        x = main.ball().pos.x  #x-coordinate of ball's position
        y = main.ball().pos.y  #y-coordinate of ball's position
        #us = main.our_robots()
        #them = main.their_robots()
        width = constants.Field.Width
        base_width = -(width / 2)
        length = constants.Field.Length

        their_end = (length * 7) / 8
        our_end = length / 8
        left = base_width + (width / 3)
        right = base_width + (width * 2) / 3
        dist = (
            length / 9
        )  # since we are creating a circle around the ball position on which to place collecting robots, this is the radius of the circle. Currently 1m

        pos1 = None
        pos2 = None
        message = None

        if (y >= their_end):
            if (x <= left):  # left corner of attacking half
                message = "pileup in attacking left corner"
                pos1 = robocup.Point(x, y - dist)
            elif (x >= right):  #right corner of attacking half
                message = "pileup in attacking right corner"
                pos1 = robocup.Point(x, y - dist)
            else:  #attacking box
                message = "pileup in illegal zone. let go immediately."
        elif (y <= our_end):
            if (x <= left):  #left corner of defending half
                message = "pileup in defending left corner"
                pos1 = robocup.Point(x, y + dist)
            elif (x >= right):  #right corner of defending half
                message = "pileup in defending right corner"
                pos1 = robocup.Point(x, y + dist)
            else:  #defending box
                message = "pileup in illegal zone. this should never occur."
        else:
            half_dist = dist / 2  #pythagorean theorem purposes
            if (x <= left):  #left flank
                message = "pileup on left flank"
                pos1 = robocup.Point(x + math.sqrt(half_dist),
                                     y + math.sqrt(half_dist))
                pos2 = robocup.Point(x + math.sqrt(half_dist),
                                     y - math.sqrt(half_dist))
            elif (x >= right):  #right flank
                message = "pileup in right flank"
                pos1 = robocup.Point(x - math.sqrt(half_dist),
                                     y + math.sqrt(half_dist))
                pos2 = robocup.Point(x - math.sqrt(half_dist),
                                     y - math.sqrt(half_dist))
            else:  #center of field
                option = random.randint(0, 1)
                if (option == 0):
                    message = "pileup in center of field - option 1"
                    pos1 = robocup.Point(x - math.sqrt(half_dist),
                                         y - math.sqrt(half_dist))
                    pos2 = robocup.Point(x + math.sqrt(half_dist),
                                         y + math.sqrt(half_dist))
                else:
                    message = "pileup in center of field - option 2"
                    pos1 = robocup.Point(x - math.sqrt(half_dist),
                                         y + math.sqrt(half_dist))
                    pos2 = robocup.Point(x + math.sqrt(half_dist),
                                         y - math.sqrt(half_dist))

        if (pos1 != None and pos2 != None):
            self.add_subbehavior(
                skills.move.Move(pos1), 'move1', required=True)
            self.add_subbehavior(
                skills.move.Move(pos2), 'move2', required=True)
            self.standbyBot1 = self.subbehavior_with_name('move1').robot
            self.standbyBot2 = self.subbehavior_with_name('move2').robot
        elif (pos1 != None):
            self.add_subbehavior(skills.move.Move(pos1), 'move', required=True)
            self.standbyBot1 = self.subbehavior_with_name('move').robot
        print(message)
