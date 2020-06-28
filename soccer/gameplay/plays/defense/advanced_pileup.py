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
        us = main.our_robots()
        them = main.their_robots()
        radius = constants.Ball.Radius
        range = 20 * radius

        c, pos = evaluation.position.ball_vicinity(us, them, range)
        print(c, pos)

        pos1 = None
        pos2 = None
        their_end = (length * 7)/8
        our_end = length/8
        left = base_width + (width/3)
        right = base_width + (width*2)/3
        dist = (length / 9)

        #initial positioning
        if (pos == -math.pi or pos == 0):
            if (ball.x < right and ball.x > left):
                pos1 = robocup.Point(ball.x - dist, ball.y)
                pos2 = robocup.Point(ball.x + dist, ball.y)
            elif (ball.x >= right):
            	pos1 = robocup.Point(ball.x - dist, ball.y)
            else: 
            	pos1 = robocup.Point(ball.x + dist, ball.y)
        elif (pos == math.pi / 2 or pos == -(math.pi / 2)):
        	if (ball.y < their_end and ball.y > our_end):
        		pos1 = robocup.Point(ball.x, ball.y - dist)
                pos2 = robocup.Point(ball.x, ball.y + dist)
            elif (ball.y >= their_end):
            	pos1 = robocup.Point(ball.x, ball.y - dist)
            else: 
            	pos1 = robocup.Point(ball.x, ball.y + dist)
        else: 
            tangent = math.tan(pos)
            if tangent > 0: # tangent = y over x
                x_sq = 1 #x^2 + y^2 = 1
                y_sq = tangent * tangent
                factor = x_sq + y_sq #to get the sum to 1
                x = math.sqrt(x_sq / factor)
                y = math.sqrt(y_sq / factor)
                pos1 = robocup.Point(ball.x - x, ball.y - y)
                pos2 = robocup.Point(ball.x + x, ball.y + y)
            else: # tangent = x over y
                y_sq = 1 #x^z + y^2 = 1
                x_sq = tangent * tangent
                factor = x_sq + y_sq
                x = math.sqrt(x_sq / factor)
                y = math.sqrt(y_sq / factor)
                pos1 = robocup.Point(ball.x - x, ball.y + y)
                pos2 = robocup.Point(ball.x + x, ball.y - y)
         

        left_end = - (width / 2)
        right end = width / 2
        our_edge = 0
        their_edge = length

        if pos1 != None:
            if pos1.x <= left_end or pos1.x >= right_end or pos1.y <= our_edge or pos1.y >= their_edge:
            	pos1 = None
            elif (pos1.y >= our_end and pos1.x >= left and pos1.x <= right):
            	pos1 = None
            else:
            	self.add_subbehavior(skills.move.Move(pos1), 'move1', required=True)
                self.standbyBot1 = self.subbehavior_with_name('move1').robot

        if pos2 != None:
            if pos2.x <= left_end or pos2.x >= right_end or pos2.y <= our_edge or pos2.y >= their_edge:
            	pos2 = None
            elif (pos2.y >= our_end and pos2.x >= left and pos2.x <= right):
            	pos2 = None
            else:
            	self.add_subbehavior(skills.move.Move(pos2), 'move2', required=True)
                self.standbyBot2 = self.subbehavior_with_name('move2').robot

