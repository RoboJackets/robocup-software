import robocup
import constants
import play
import skills
import tactics
import main
import evaluation.position
import math
import random


class AdvancedPileup(play.Play):
    def __init__(self):
        super().__init__(continuous=False)
        ball = main.ball().pos
        radius = constants.Ball.Radius
        range = 20 * radius

        c, pos = evaluation.position.ball_vicinity(range)

        pos1 = None
        pos2 = None

        left_end = -(width / 2)
        right_end = width / 2
        our_edge = 0
        their_edge = length

        their_end = (length * 7) / 8
        our_end = length / 8
        left = left_end + (width / 3)
        right = right_end - (width / 3)
        dist = (length / 9)

        #initial positioning - I will keep this here until we have been able to test, since I am unable to test on my computer
        '''
        if pos == -math.pi or pos == 0:
            if (ball.x < right and ball.x > left):
                pos1 = robocup.Point(ball.x - dist, ball.y)
                pos2 = robocup.Point(ball.x + dist, ball.y)
            elif (ball.x >= right):
                pos1 = robocup.Point(ball.x - dist, ball.y)
            else:
                pos1 = robocup.Point(ball.x + dist, ball.y)
        elif pos == math.pi / 2 or pos == -(math.pi / 2):
            if (ball.y < their_end and ball.y > our_end):
                pos1 = robocup.Point(ball.x, ball.y - dist)
                pos2 = robocup.Point(ball.x, ball.y + dist)
            elif (ball.y >= their_end):
                pos1 = robocup.Point(ball.x, ball.y - dist)
            else:
                pos1 = robocup.Point(ball.x, ball.y + dist)
        else:
            tangent = math.tan(pos)
            if tangent > 0:  # tangent = y over x
                x_sq = 1  #x^2 + y^2 = 1
                y_sq = tangent * tangent
                factor = x_sq + y_sq  #to get the sum to 1
                x = math.sqrt(x_sq / factor)
                y = math.sqrt(y_sq / factor)
                pos1 = robocup.Point(ball.x - x, ball.y - y)
                pos2 = robocup.Point(ball.x + x, ball.y + y)
            else:  # tangent = x over y
                y_sq = 1  #x^z + y^2 = 1
                x_sq = tangent * tangent
                factor = x_sq + y_sq
                x = math.sqrt(x_sq / factor)
                y = math.sqrt(y_sq / factor)
                pos1 = robocup.Point(ball.x - x, ball.y + y)
                pos2 = robocup.Point(ball.x + x, ball.y - y)
        '''

        delta1 = robocup.Point.direction(angle)
        pos1 = main.ball().pos - delta1
        pos2 = main.ball().pos + delta1

        if pos1 != None:
            if constants.Field.FieldRect.contains_point(pos1):
                if (pos1.x >= left and pos1.x <= right and
                    (pos1.y >= their_end or pos1.y <= our_end)):
                    self.add_subbehavior(
                        skills.move.Move(pos1), 'move1', required=True)
                    self.standbyBot1 = self.subbehavior_with_name(
                        'move1').robot
                else:
                    pos1 = None  #in field but in illegal position
            else:
                pos1 = None  #not even in field

        if pos2 != None:
            if constants.Field.FieldRect.contains_point(pos2):
                if (pos2.x >= left and pos2.x <= right and
                    (pos2.y >= their_end or pos2.y <= our_end)):
                    self.add_subbehavior(
                        skills.move.Move(pos1), 'move2', required=True)
                    self.standbyBot2 = self.subbehavior_with_name(
                        'move2').robot
                else:
                    pos2 = None  #in field but in illegal position
            else:
                pos2 = None  #not in field

        # delta1 = robocup.Point.direction(angle)
        # pos1 = main.ball().pos - delta1
