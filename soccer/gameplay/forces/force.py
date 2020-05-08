from abc import ABC, abstractmethod
import robocup
import force_sample
import composite_force
import copy

class Force(ABC):

    name = "force" 
    compositable = False


    @abstractmethod
    def sample(self, sample_point):
        return ForceSample(robocup.Point(0,0), sample_point)

    def __str__(self):
        return name

    def __add__(self, other):
        if(other.compositable):
            newComposite = copy.deepcopy(other)  
            newComposite.addForce(copy.deepcopy(self))
            return newComposite
        else:
            return composite_force(forces=[self, other], merge_function=lambda x : sum(x))

    

