
from forces import force_utils
from forces import composite_force
from forces import direction
from forces import point_force


#A force origonating from  number of points symmetrically
# So basically they all need to have the same responce function
class PointsForce(composite_force.CompositeForce):
  
    #Pretty important to note that these all have to be lambda functions
    points = list()

    ## The responce function 
    #responce_function = lambda s, x : force_utils.log_responce()
    responce_direction = direction.Direction.PUSH
    responce_function = lambda s, x : force_utils.log_responce(x, 2.0, 1.0)

    ##Going along with the idea that this is a composite force, you can override anything in composite force here
    merge_function = lambda s, x : sum(x)

    initalized = False

    ##This class cannot be composited because of the need to activly manage the contained forces
    compositable = False
    
    def __init__(self): 
        pass

    #Note that each point needs to be a lambda
    #This might need a naming overhaul
    def add_point(self, point):
        self.initalized = False
        self.points.append(point)

    #Adds a point directly rather than it being a lambda
    def add_point_raw(self, point):
        self.initalized = False
        self.points.append(lambda sample_point : point)
    
    def initalize(self):
        self.initalized = True
        new_forces = list()
        for g in self.points:
            new_force = point_force.PointForce()
            new_force.point_lam = g
            new_force.responce_function = self.responce_function
            new_force.responce_direction = self.responce_direction
            new_forces.append(new_force)
        self.forces = new_forces

    def sample(self, point):
        if (not self.initalized):
            self.initalize()
        samples = super().sample(point)
        return samples







