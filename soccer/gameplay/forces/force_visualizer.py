import main
import robocup
import math
from numpy import arange
from forces import force_utils


##
#
# Visualizes a force field
#
#
class ForceVisualizer():


    force_field = None

    #Some parameters for how the field is to be visualized

    ## A lambda for the line length based on the sample magnitude
    line_length = lambda s, x : x
    ## A lambda for the color based on the sample magnitude
    color = lambda s, mag : (255, 0, 0)   
    circle_radius = lambda s, mag : 0.05

    context = None

    def __init__(self, force_field = None):
        self.force_field = force_field
        self.context = main.context()


    ##
    # Visualize a single sample
    #
    def visualizeSample(self, sample):
        mag = sample.vector.mag()
        vectorEnd = sample.origin + robocup.Point(sample.vector.x, sample.vector.y)
        self.context.debug_drawer.draw_circle(sample.origin, self.circle_radius(mag), self.color(mag), "layer?")
        self.context.debug_drawer.draw_segment_from_points(sample.origin, vectorEnd, self.color(mag), "hat")

    ##
    # Sets the colors to be a thermal colorscheme
    #
    def set_color_thermal(self, exp_min=0, exp_max=10):
        self.color = lambda mag : force_utils.thermal_rgb_convert(mag, exp_min, exp_max)

    ##
    #
    # Visualize the field
    #
    def visualizeField(self, generate=True):
        if(generate):
            self.force_field.generate()

        for g in self.force_field.samples:
            self.visualizeSample(g)



