import robocup
import forces
from forces import force
from forces import force_sample


class ConstantForce(force.Force):


    def __init__(self, vector=robocup.Point(1,1)):
        self.vector = vector

    def sample(self, sample_point):
        return force_sample.ForceSample(self.vector,sample_point)

