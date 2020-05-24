import play
import behavior
import robocup
import constants

from forces import force_visualizer
from forces import force
from forces import constant_force
from forces import test_robot_force
from forces import linear_our_robot_force
from forces import edge_force
from forces import composite_force
from forces import force_field

##
# A demo/base test play for visualizing forces using the force
# visualizer
#
#
#
class ForceVisualize(play.Play):
    

    ##This point will be the bottom left corner of the field
    #corner = robocup.Point(0, constants.Field.Length / 2)
    corner = robocup.Point(-1 * (constants.Field.Width / 2),0)

    ##Set how you want the points drawn here
    x_size = 6.0
    y_size = 10.0
    interval = 0.5

    ##There is a default scale factor, but you can also set it here
    scaleFactor = 0.3

    ##You can swap out the force you want to visualize here
    #force = constant_force.ConstantForce(robocup.Point(1.2,1.3))
    #forceA = linear_our_robot_force.LinearOurRobotForce()
    #forceB = edge_force.EdgeForce()
    #force = composite_force.CompositeForce([forceA, forceB])


    def __init__(self):
        super().__init__(continuous=False)

        cForce = constant_force.ConstantForce(robocup.Point(1.2,1.3))

        field = force_field.ForceField(force=cForce)
        field.set_sample_grid(corner=self.corner, x_range=self.x_size, y_range=self.y_size, step=self.interval)


        #Create the visualizer object
        self.visualizer = force_visualizer.ForceVisualizer(force_field = field)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')


    def execute_running(self):
        self.visualizer.visualizeField(generate=True)
