
import robocup
import forces
from forces import force
import main
from abc import abstractmethod
import numpy as np


##
#
# So uhhhh, this is kind of shitty right now but that's what we're aiming for so it's ok
#
class EdgeForce(force.Force):

    def sample(self, point):


        #print(robocup.Field_Dimensions.CurrentDimensions.FieldRect)
        #print(robocup.Field_Dimensions.CurrentDimensions.FieldBorders[0])
        #print(robocup.Field_Dimensions.CurrentDimensions.FieldBorders[0])

        dists = [x.dist_to(point) for x in robocup.Field_Dimensions.CurrentDimensions.FieldBorders]
        ind = np.argmin(dists) 
        vec = None
        if(dists[ind] == 0):
            return robocup.Point(0,0)

        val = 1 / dists[ind]
        #Right now this is assuming that the list of borders
        #will always

        if(ind == 0):
            vec = robocup.Point(val,0)
        elif(ind == 1):
            vec = robocup.Point(0,-1*val)
        elif(ind == 2):
            vec = robocup.Point(-1*val,0)
        elif(ind == 3):
            vec = robocup.Point(0,val)

        return vec
    


