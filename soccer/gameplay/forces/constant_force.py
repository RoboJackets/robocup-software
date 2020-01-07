import robocup
import forces
from forces import force

class ConstantForce(force.Force):


    def __init__(self, vector=robocup.Point(1,1)):
        self.vector = vector

    def sample(self, clipped = True):
        return self.vector

