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
        forceSample = self.force.sample(point)

        drawVector = forceSample.origin + robocup.Point(forceSample.vector.x * self.scaleFactor, forceSample.vector.y * self.scaleFactor)
        self.context.debug_drawer.draw_circle(forceSample.origin, 0.05, (0,0,0), "layer?")
        self.context.debug_drawer.draw_segment_from_points(forceSample.origin, drawVector, (255, 0, 0), "hat")

    def fieldVisualize(self, corner, xSize, ySize, interval):
    
        points = list()

        for x in arange(0, xSize, interval):
            for y in arange(0, ySize, interval):
                points.append(robocup.Point(corner + robocup.Point(x, y)))

        for g in points:
            self.pointVisualize(g)





