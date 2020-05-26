
import robocup
import main
from abc import abstractmethod
import copy
from forces import force
from forces import force_sample


class CompositeForce(force.Force):

    forces = list()
    compositable = True

    #Kill me
    #merge_function = lambda s, x : sum(x,start=force_sample.ForceSample(origin=None, vector=robocup.Point(0,0)))
    #Wait, I've found something better, leave me alive
    merge_function = lambda s, x : sum(x)
   

    def __init__(self, force_list=None, merge_function=None):
        if(force_list is not None):
            self.forces = force_list
        else:
            self.forces = list()
        if(merge_function is not None):
            self.merge_function = merge_function

    """
    def __add__(self, other):
        newComposite = copy.deepcopy(self) 
        if(other.compositable):
           newComposite.forces.append(other.forces) 
           return newComposite
        else:
            newComposite.append(other)
            return newComposite
    """


    def addForce(self, force):
        self.forces.append(force)

    def sample(self, point):
        retForce = robocup.Point(0,0)
        sample_list = [x.sample(point) for x in self.forces]
        if(len(sample_list) == 0):
            return force_sample.ForceSample(origin=point, vector=robocup.Point(0,0))
        retSample = self.merge_function(sample_list) 
        return retSample
    


