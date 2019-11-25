from abc import ABC, abstractmethod
import robocup

class Force(ABC):

    ## A maximum magnitude for this force
    highClip = None


    @abstractmethod
    def sample(self, clipped = True):
        return robocup.Point(0,0) 

    def sampleArea(self, center, xSize, ySize, interval, clipped = True):
        pass

    def sampleAreaMag(self, center, xSize, ySize, interval, clipped = True):
        pass

    ##Finds the minimum of a sampled grid of points
    def areaMin(self, center, xSize, ySize, interval):
        pass
