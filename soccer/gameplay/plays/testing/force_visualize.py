import play
import behavior
import robocup
import constants
import math
import main

from forces import force_visualizer
from forces import force
from forces import constant_force
from forces import test_robot_force
from forces import linear_our_robot_force
from forces import edge_force
from forces import composite_force
from forces import force_field
from forces import directional_force
from forces import force_sample
from forces import point_force
from forces import force_utils
from forces import robot_force
from forces import direction


##
# A demo/base test play for visualizing forces using the force
# visualizer
#
#
#
class ForceVisualize(play.Play):
    

    ##This point will be the bottom left corner of the field
    #corner = robocup.Point(0, constants.Field.Length / 2)
    plot_corner = robocup.Point((-1 * (constants.Field.Width / 2)),0)

    ##Set how you want the points drawn here
    x_size = 6.0# - 0.1
    y_size = 10.0# - 0.1
    interval = 0.5

    ##There is a default scale factor, but you can also set it here
    #scaleFactor = 0.3

    ##You can swap out the force you want to visualize here
    #force = constant_force.ConstantForce(robocup.Point(1.2,1.3))
    #forceA = linear_our_robot_force.LinearOurRobotForce()
    #forceB = edge_force.EdgeForce()
    #force = composite_force.CompositeForce([forceA, forceB])

    systemState = None

    def __init__(self):
        super().__init__(continuous=False)


        self.systemState = main.system_state()
        
        #a = force_sample.ForceSample(robocup.Point(1.2,2.3), robocup.Point(2,4))
        #b = force_sample.ForceSample(robocup.Point(1.2,2.3), robocup.Point(2,4))
        #my_list = [a,b]
        #print(sum(my_list))


        #A constant force
        cForce = constant_force.ConstantForce(robocup.Point(1,1))
        
        #A directional force, with some options specified
        dForce = directional_force.DirectionalForce(constant_dir=None,degrees=True)
        dForce.direction = lambda x, y : math.sin(x)
        dForce.magnitude = lambda x, y : math.cos(y)

        #A composite force created from the two prior forces
        comp_force = composite_force.CompositeForce()
        comp_force.addForce(cForce)
        comp_force.addForce(dForce)

        pForce = point_force.PointForce()
        pForce.point_lam = lambda sample_point : self.systemState.ball.pos
        #pForce.responce_function = lambda x : force_utils.log_responce(x, 2.0, 3.0)

        #An edge force, that repels from the edges of the field
        eForce = edge_force.EdgeForce()
       
        #A composite force of the edge force and the directional force
        comp_force2 = composite_force.CompositeForce()
        comp_force2.addForce(pForce)
        comp_force2.addForce(eForce)
      
        #rForce = robot_force.RobotForce()

        pForce2 = point_force.PointForce()
        pForce2.point_lam = lambda sample_point : self.systemState.our_robots[0].pos

        comp_force3 = composite_force.CompositeForce()
        comp_force3.addForce(pForce2)
        comp_force3.addForce(pForce)
        pForce.responce_direction = direction.Direction.PULL

        field = force_field.ForceField(force=comp_force3)
        field.set_sample_grid(corner=self.plot_corner, x_range=self.x_size, y_range=self.y_size, step=self.interval)


        #Create the visualizer object
        self.visualizer = force_visualizer.ForceVisualizer(force_field = field)
   
        #self.visualizer.set_color_thermal(exp_min=0,exp_max=1)
        #self.visualizer.line_length = lambda mag : 0.2


        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')


    def execute_running(self):
        self.visualizer.visualizeField(generate=True)
