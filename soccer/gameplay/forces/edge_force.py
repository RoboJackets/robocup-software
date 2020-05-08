
import robocup
import forces
from forces import force
import main
from abc import abstractmethod
import numpy as np
from forces import force_utils
from forces import force_sample
from forces import direction
from forces import point
from forces import points_force


##
# Should this serve as a base class or should it have functionality all it's own?
#
class EdgeForce(points_force.PointsForce):

    borders = robocup.Field_Dimensions.CurrentDimensions.FieldBorders 

    responce_function = lambda x : force_utils.trig_responce(x, 2.0, 1.0, 2.0)
    responce_type = direction.Direction.PUSH
    merge_function = lambda x : max(x)

    ##I don't know if this is horrible or butiful
    points = [lambda x : border.nearest_point(x) for border in self.borders]



    """
    def sample(self, point):
        self.points = [x.nearest_point(point) for x in borders] #If the borders aren't segments this may not work
        super(point)
    """

    """
    #This is the sketch way of doing this, I guess what I should do is make this a points force?
    #So I've got an implementation here, with a lambda for customization
    def sample(self, point):

        dists = [x.dist_to(point) for x in borders]
        
        #ind = np.argmin(dists) 
        vec = None

        if(dists[ind] == 0):
            return robocup.Point(0,0)

        #should be a list of the magnitudes
        mags = [self.responce_function(x) for x in dists]


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
   """ 


