import play
import behavior
import tactics.line_up
import robocup
import constants
import forces.force_visualizer
import forces.force
import forces.constant_force


class ForceVisualize(play.Play):
    

    center = robocup.Point(2,2)
    
    x_points = 10
    y_points = 10
    
    x_size = 2.0
    y_size = 2.0

    def __init__(self):
        super().__init__(continuous=False)

        self.force = constant_force.ConstantForce(robocup.Point(1.2,1.3))
        self.visualizer = force_visualizer.ForceVisualizer(self.force)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')
       

    def on_enter_running(self):
        pass


    def execute_running(self):
        self.visualizer.pointVisualize(center)
