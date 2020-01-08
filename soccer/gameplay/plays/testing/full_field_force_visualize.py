import robocup
import constants
from plays.testing import force_visualize
from forces import constant_force


##
# this is just the force visualize test play set to visualize for the whole field
#
class FullFieldForceVisualize(force_visualize.ForceVisualize):
    
    corner = robocup.Point(-1 * (constants.Field.Width / 2), 0)
   
    x_size = constants.Field.Width + 0.01
    y_size = constants.Field.Length + 0.01
    interval = 0.5

    scaleFactor = 0.3

    ##You can swap out the force you want to visualize here
    force = constant_force.ConstantForce(robocup.Point(1.2,1.3))




