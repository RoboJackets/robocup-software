
import robocup
import forces
from forces import force
import main
from abc import abstractmethod
import numpy as np
from forces import force_utils
from forces import force_sample
from forces import direction
from forces import point_force
from forces import points_force


##
# Should this serve as a base class or should it have functionality all it's own?
#
class EdgeForce(points_force.PointsForce):

    borders = robocup.Field_Dimensions.CurrentDimensions.FieldBorders 

    #responce_function = lambda s, x : force_utils.trig_responce(x, 2.0, 1.0, 2.0)
    responce_type = direction.Direction.PUSH
    #A max merge function probably makes more sense here but it was breaking
    merge_function = lambda s, x : sum(x)

    responce_function = lambda s, x : force_utils.log_responce(x, 2.0, 3.0, 0.1)
    
    ##I don't know if this is horrible or butiful
    #points = [lambda x : border.nearest_point(x) for border in borders]
    #Why these arent the same I have absolutly no idea
    #points = [lambda x : border.nearest_point(x) for border in borders]
    #test_point = lambda x : borders[0].nearest_point(x)
    
    #This was such a cute one liner and now look at it :(
    points = [lambda x : robocup.Field_Dimensions.CurrentDimensions.FieldBorders[0].nearest_point(x),
              lambda x : robocup.Field_Dimensions.CurrentDimensions.FieldBorders[1].nearest_point(x),
              lambda x : robocup.Field_Dimensions.CurrentDimensions.FieldBorders[2].nearest_point(x),
              lambda x : robocup.Field_Dimensions.CurrentDimensions.FieldBorders[3].nearest_point(x)]
 


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


