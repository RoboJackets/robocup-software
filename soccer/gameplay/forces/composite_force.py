
import robocup
import forces
from forces import force
import main
from abc import abstractmethod

class CompositeForce(force.Force):

    forces = list()

    merge_function = lambda x : sum(x)

    def __init__(self, forces=None, merge_function=None):
        if(force_list is not None):
            self.force_list = force_list
        if(merge_function is not None):
            self.merge_function = merge_function

    def addForce(self, force):
        force_list.append(force)

    def sample(self, point):
        retForce = robocup.Point(0,0)
        sample_list = list()
        for g in self.force_list:
            sampleg.sample(point)
        
        sample_list = [x.sample(point) for x in self.force_list]
        retForce = merge_function(sample_list) 
        

        return retForce





