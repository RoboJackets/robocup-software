import standard_play
import forces.force
import robocup
import behavior
import main
import forces.force_visualizer


class TestForcePlay(standard_play.StandardPlay):
   
    def __init__(self):
        super().__init__(continuous=False)
        self.context = main.context()

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            "immediately")
    @classmethod    
    def score(cls):
        return 1.0

    def execute_running(self):
        self.context.debug_drawer.draw_text("This is the force testing play", robocup.Point(0, 0.5),(0, 0, 0), "hat")






