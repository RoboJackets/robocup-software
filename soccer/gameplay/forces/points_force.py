
from forces import force_utils
from forces import composite_force
from forces import direction

#A force origonating from  number of points symmetrically
#
class PointsForce(composite_force.CompositeForce):
   
    points = list()

    ## The responce function 
    responce_function = lambda x : force_utils.log_responce()
    responce_direction = direction.Direction.PUSH

    merge_function = lambda x : sum(x)

    initalized = False

    ##This class cannot be composited because of the need to activly manage the contained forces
    compositable = False
    
    def __init__(self): 
        pass

    def add_point(self, point):
        self.initalized = False
        self.points.append(point)
    
    def initalize(self):
        self.initalized = True
        forces = list()
        for g in points:
            #Generate a corasponding force and add it to the list
            raise NotImplemented("I need to do this")

    def sample(self, point):
        if (not self.initalized):
            self.initalize()
        return super(point)







