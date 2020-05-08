
import robocup
import forces
from forces import force
import main
from abc import abstractmethod
import copy
from forces import points_force

class CompositeForce(force.Force):

    forces = list()
    compositable = True


    merge_function = lambda x : sum(x)

    def __init__(self, forces=None, merge_function=None):
        if(force_list is not None):
            self.force_list = force_list
        if(merge_function is not None):
            self.merge_function = merge_function

    def __add__(self, other):
        newComposite = copy.deepcopy(self) 
        if(other.compositable):
           newComposite.forces.append(other.forces) 
           return newComposite
        else:
            newComposite.append(other)
            return newComposite

    def addForce(self, force):
        force_list.append(force)

    def sample(self, point):
        retForce = robocup.Point(0,0)
        sample_list = list()
        sample_list = [x.sample(point) for x in self.force_list]
        retSample = self.merge_function(sample_list)   
        return retSample
    


