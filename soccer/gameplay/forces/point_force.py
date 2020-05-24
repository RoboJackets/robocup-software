from forces import force
from forces import force_utils
from forces import force_sample
from forces import direction


##
#
# A force that eminates from a point
#
class PointForce(force.Force):

    #Does this need to be a lambda??? if you are doing the location of a robot than it could be like a refrence but that's kind of sketch
    ## A lambda that returns the point from which the force is originating as a function of the sample point
    point = lambda sample_point : robocup.Point(0,0)

    responce_function = lambda x : force_utils.log_responce(x, 2.0, 1.0)
    responce_direction = direction.Direction.PUSH 

    ##
    # @param point_function a lambda function with no arguments that returns the point of origin of the force
    #
    #
    def __init__(self, point_function = None, responce_function=None, responce_direction=None):
        if(point_function is not None):
            self.point = point_function
        if(responce_function is not None):
            self.responce_function = responce_function
        if(responce_direction is not None):
            self.responce_direction = responce_direction

    def sample(self, sample_point):
        vec = force_utils.push(self.point(), sample_point)
        vec = force_utils.vec_scale(vec,responce_function(norm.mag())) 

        if(self.responce_direction is direction.Direction.PUSH):
            vec = force_utils.vec_invert(vec)
           
        return force_sample.ForceSample(vector=vec, origin=sample_point)


