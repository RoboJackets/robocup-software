import sys
sys.path.insert(1, "../../stp")
import rc
sys.path.append("/utils")
import constants

class Chip:
    
    def distance(loc1, loc2):
        return sqrt(abs(loc1[1] - loc2[1])**2 + abs(loc1[0] - loc2[0])**2)

	def chippable_robots(ball, their_robots):
	    bpos = ball.pos
        dist = distance(bpos, r.pos[:-1])
        return [r for r in their_robots if (dist > constants.OurChipping.MIN_CARRY and dist < constants.OurChipping.MAX_CARRY)]
		#return None

	#ball = rc.Ball([0,4.5],[0, -1], True)
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
