from abc import ABC, abstractmethod
import robocup

class Force(ABC):

    @abstractmethod
    def sample(self):
        return robocup.Point(0,0) 

    def sampleArea(self):
        pass

    def sampleAreaMag(self):
        pass

    #Finds the minimum of a sampled grid of points
    def areaMin(self):
        pass
