import robocup
import forces
from forces import force
from forces import force_sample


##
#
# A force that is simply a constant vector
#
#
class ConstantForce(force.Force):

    def __init__(self, vector=robocup.Point(1,1)):
        self.vector = vector

    def sample(self, sample_point):
        return force_sample.ForceSample(vector = self.vector, origin = sample_point)

    """
    def __add__(self, other):
        newForce = copy.deepcopy(self)
        newForce.vector += other.vector
        return newForce
    """

