import main
import robocup
import constants
import math

'''calculates the distance of a robot to the ball'''
def distance(robot):
	x = abs(main.ball().pos.x - robot.pos.x)
	y = abs(main.ball().pos.y - robot.pos.y)
	dist = math.sqrt((x*x) + (y*y))
	return dist
	
'''returns all robots, as well as their average positions, that are within a given radius of the ball'''
def ball_vicinity(us, them, radius):
    contending = []
    #contending_us = []
    #contending_them = []

    for u in us:
    	if distance(u) <= radius:
    		contending.append(u)
    		#contending_us.append(u)

    for t in them:
    	if distance(t) <= radius:
    		contending.append(t)
    		#contending_them.append(t)

    cpos = 0 #position sum to calculate average position of robots (-pi to pi)
   
    for w in contending:
        x = w.pos.x - main.ball().pos.x
        y = w.pos.y - main.ball().pos.y

        cpos += math.atan2(y,x)


    if len(contending) == 0:
    	avgpos = 0
    else:
        avgpos = cpos / len(contending)

    return contending, avgpos