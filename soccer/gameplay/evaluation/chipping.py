import main 
import robocup
import constants
import math

#
# Returns an array of their robots that we can chip over based on the current ball position 
# this only operates on the distances from the ball to each robot and the Chip distances in constants.py
#

def chippable_robots(bp= None):
    if bp is None:
        bp = main.ball().pos

    robots = [rob for rob in main.system_state().their_robots if \
                (rob.pos-bp).mag() > constants.OurChipping.MIN_CARRY and \
                (rob.pos-bp).mag() < constants.OurChipping.MAX_CARRY ]

    if robots is None or not isinstance(robots, list):
        return []
    else:
        return robots
 