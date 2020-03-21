
import robocup
import forces
from forces import force
import main
from abc import abstractmethod

class CompositeForce(force.Force):

    force_list = list() 


    def __init__(self, force_list=None):
        if(force_list is not None):
            self.force_list = force_list
            

    def addForce(self, force):
        force_list.append(force)

    def sample(self, point):
        retForce = robocup.Point(0,0)
        for g in self.force_list:
            retForce += g.sample(point)

        return retForce





