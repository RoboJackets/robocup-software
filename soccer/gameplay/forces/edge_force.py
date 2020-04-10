
import robocup
import forces
from forces import force
import main
from abc import abstractmethod
import numpy as np
from forces import force_utils
from forces import force_sample
from forces import direction

##
# Should this serve as a base class or should it have functionality all it's own?
#
class EdgeForce(force.Force):

    borders = robocup.Field_Dimensions.CurrentDimensions.FieldBorders 

    responce_function = lambda x : force_utils.trig_responce(x, 2.0, 1.0, 2.0, clipLow=0.15)
    responce_type = direction.Direction.PUSH

    #So I've got an implementation here, with a lambda for customization
    def sample(self, point):

        dists = [x.dist_to(point) for x in robocup.Field_Dimensions.CurrentDimensions.FieldBorders]
        ind = np.argmin(dists) 
        vec = None

        if(dists[ind] == 0):
            return robocup.Point(0,0)
 
        mag = self.responce_function(dists[ind])

        if(ind == 0):
            vec = robocup.Point(mag,0)
        elif(ind == 1):
            vec = robocup.Point(0,-1*mag)
        elif(ind == 2):
            vec = robocup.Point(-1*mag,0)
        elif(ind == 3):
            vec = robocup.Point(0,mag)

        if self.responce_type is not direction.Direction.PUSH:
            vec = force_utils.vec_invert(vec)

        return force_sample.ForceSample(vec, point)
    


