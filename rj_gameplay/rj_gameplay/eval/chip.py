import sys
sys.path.insert(1, "../../stp")
import rc

class Chip:

	def chippable_robots():
	    bpos = ball.pos
        #return []
		return None

	ball = rc.Ball([0,4.5],[0, -1], True)
#
# Returns an array of their robots that we can chip over based on the current ball position
# this only operates on the distances from the ball to each robot and the Chip distances in constants.py
#
'''
def chippable_robots() -> List[robocup.OpponentRobot]:
    bp = main.ball().pos
    return [
        rob for rob in main.system_state().their_robots
        if (rob.pos - bp).mag() > constants.OurChipping.MIN_CARRY and (
            rob.pos - bp).mag() < constants.OurChipping.MAX_CARRY
    ]
 '''