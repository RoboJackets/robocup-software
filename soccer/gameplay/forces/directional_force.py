from forces import force
from forces import force_utils
import robocup
import math
from forces import force_sample

##
# A class for a force where the direction is decoupled from the magnatude, which has a responce on the x and y functions
#
#
class DirectionalForce(force.Force):
 
    #If you wanted to be a real edgelord you could change the direction on every sample 
    #and make this emulate the functionality of another force. But don't do that?

    ##A lambda that returns a direction when passed the sample point
    #When I define a lambda here I have to have a self parameter but not other places???
    direction = lambda s, x, y : 0.0

    ##A lambda that will produce ab ekenebt 
    #x_responce = lambda x : 0
    #y_responce = lambda y : y

    ## A lambda function that returns the magnitude of the force 
    magnitude = lambda s, x, y : y

    def __init__(self, constant_dir=None, degrees=False):
        if(constant_dir is not None):
           self.set_constant_direction(constant_dir, degrees) 

    ##
    # If you want a constant direction, this function will do that for you 
    #
    def set_constant_direction(self, direction, degrees=False):  
        if(degrees):
            direction = math.radians(direction)
        #Note, when I define the function here I don't have an self parameter and it works????
        self.direction = lambda x, y : direction

    def sample(self, point):
        direction = self.direction(point.x, point.y)
        direction_vector = robocup.Point(math.cos(direction), math.sin(direction))
        mag = self.magnitude(point.x, point.y)
        return force_sample.ForceSample(vector=direction_vector * mag, origin=point)

