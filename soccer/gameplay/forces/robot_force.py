import robocup
import forces
from forces import force
import main
from abc import abstractmethod

class RobotForce(force.Force):

    ourRobots = None
    theirRobots = None
    systemState = None


    def __init__(self):
        self.systemState = main.system_state()
        self.updateVisibility()        


    def updateVisibility(self):
        if(self.systemState is None):
            self.systemState = main.system_state()

        self.ourRobots = list()
        self.theirRobots = list()
        
        for g in self.systemState.our_robots:
            self.ourRobots.append(g)
        for g in self.systemState.their_robots:
            self.theirRobots.append(g)

    @abstractmethod
    def sample(self, clipped = True):
        return robocup.Point(0,0)




