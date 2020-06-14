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
        radius = constants.ball.radius
        range = 5 * radius
        c, pos = evaluation.position.ball_vicinity(us, them, radius)
        print(c, pos)