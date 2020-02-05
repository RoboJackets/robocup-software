import main
import robocup
import math
from numpy import arange

#A class for visualizing forces using the debug drawer.
class ForceVisualizer():

    def __init__(self, force, scaleFactor = 0.3):
        self.context = main.context()
        self.force = force
        self.scaleFactor = scaleFactor

    def pointVisualize(self, point):
        #self.context.debug_drawer.draw_text("This is the force visualizer",robocup.Point(0, 0.5),(0, 0, 0), "hat")
        forceSample = self.force.sample(point)
        
        #Possible should add some type of clipping, should integrate with color support
        #if(forceSample.mag() > 50):
        #    pass


        #Planning to make the segment colors correlate with something for pretty rainbows

        drawVector = point + robocup.Point(forceSample.x * self.scaleFactor, forceSample.y * self.scaleFactor)
        self.context.debug_drawer.draw_circle(point, 0.05, (0,0,0), "layer?")
        self.context.debug_drawer.draw_segment_from_points(point, drawVector, (255, 0, 0), "hat")
        #self.force.clipHigh

    def fieldVisualize(self, corner, xSize, ySize, interval):
    
        points = list()

        #This functionality is basically duplicated in force.py I think
        #It should probably just be there.

        for x in arange(0, xSize, interval):
            for y in arange(0, ySize, interval):
                points.append(robocup.Point(corner + robocup.Point(x, y)))

        for g in points:
            self.pointVisualize(g)





