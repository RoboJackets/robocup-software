from forces import force
from forces import force_utils
import robocup

#This is intended to be a force with a constant direction, but can have a scalable magnatude
#I might want to think a little bit more about how this is done
class DirectionalForce(force.Force):
 
    #If you wanted to be a real edgelord you could change the direction on every sample 
    #and make this emulate the functionality of another force.
    direction = 0.0

    x_responce = lambda x : 0
    y_responce = lambda y : y

    #Direction should be the direction you want in degrees
    def __init__(self, direction = 0.0):
        self.direction = direction

    def sample(self, point):
        base_vector = robocup.Point(0,1)
        rotated_vector = force_utils.rotate_vector(base_vector, self.direction)
        mag = x_responce(point.x) + y_responce(point.y)
        return ForceSample(rotated_vector * mag, point)

