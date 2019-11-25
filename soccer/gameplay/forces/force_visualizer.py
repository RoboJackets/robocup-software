import main

#A class for visualizing forces using the debug drawer.
class ForceVisualizer():

    def __init__(self, force):
        self.context = main.context()
        self.force = force

    def pointVisualize(self, point, force):
        self.context.debug_drawer.draw_text("This is the forcce visualizer",robocup.Point(0, 0.5),(0, 0, 0), "hat")
        self.context.debug_drawer.draw_circle()
        self.context.debug_drawer.draw_segment()
        self.force.clipHigh

    def fieldVisualize(self, corner, xSize, ySize, interval):
        pass


