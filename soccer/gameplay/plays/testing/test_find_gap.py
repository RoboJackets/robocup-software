import play
import behavior
import robocup
import main
import evaluation.shooting

class TestFindGap(play.Play):
    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            "immediately")

    def execute_running(self):
        evaluation.shooting.find_gap(max_shooting_angle=70, robot_offset=8, dist_from_point=1)