from abc import ABC, abstractmethod
import robocup

class Force(ABC):

    #So clipping needs to be figured out, idk how that should be done though
    #It should probably exist somewhere but I'm not sure if here is the best place
    #Especially if children are expected to overwrite sample


    #Some stuff needs to change here? Should a force sample return like a tuple or a specific object?


    ## A maximum magnitude for this force
    highClip = None

    @abstractmethod
    def sample(self, clipped = True):
        return robocup.Point(0,0) 

    ##
    # returns a dict of points and the sample at those points
    #  
    # UHHHHH I don't know what to do here with respect to point count vs interval
    def sampleArea(self, center, xSize, ySize, interval = None, clipped = True):
        retDict = dict()
        points = list()

        for x in numpy.linspace(0, xSize, interval):
            pass

    def sampleAreaMag(self, center, xSize, ySize, interval, clipped = True):
        pass

    ##Finds the minimum of a sampled grid of points
    def areaMin(self, center, xSize, ySize, interval):
        pass
