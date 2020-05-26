
import numpy as np
from forces import force_utils
import robocup


##
# A field of forces that can be continually generated
#
class ForceField():


    ## the list of force samples
    samples = None
    force = None 

    sample_points = list()

    def __init__(self, force=None, sample_points=list()):
        self.force = force
        self.sample_points = sample_points

    ## 
    # Sets the sample point list to the specified grid
    #
    def set_sample_grid(self, corner=robocup.Point(0,0), x_range=1, y_range=1, step=0.5):
        self.sample_points = [robocup.Point(x,y) for x in np.arange(corner.x, corner.x + x_range, step) for y in np.arange(corner.y, corner.y + y_range, step)]


    ##
    # Generate the field 
    #
    def generate(self):
        self.samples = [self.force.sample(sample_point) for sample_point in self.sample_points] 
        
    ##
    # Augment the field samples with a passed in function
    # Note that your lambda function needs to operate on force samples
    # 
    def augment(self, func = lambda x : x):
        self.samples = [func(x) for x in samples]

    ## A function to augment the force field with laziness
    def apply_lazy(self, responce = lambda x : 1 + x):
        self.augment(func = lambda x : force_utils.lazy_function(x, responce))

    ##
    # Gets the maximum sample
    #
    def get_max_sample(self):
        return max(samples)

    ##
    # Gets the minimum sample
    #
    def get_min_sample(self):
        return min(samples)


