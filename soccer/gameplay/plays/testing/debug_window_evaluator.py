import play
import behavior
import main
import robocup
import evaluation.window_evaluator
import constants
import time
import math

# This isn't a real play, but it's pretty usefug
# Turn it on and we'll draw the window evaluator stuff on-screen from the ball to our goal
class DebugWindowEvaluator(play.Play):

    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')

        self.avg_time_saved = 0.0
        self.avg_count = 0.0


    def execute_running(self):
        pt = main.ball().pos
        seg = constants.Field.OurGoalSegment

        win_eval_cpp = robocup.WindowEvaluator(main.system_state())
        win_eval_cpp.debug = True
        start = time.time()
        windows,best = win_eval_cpp.eval_pt_to_our_goal(pt)
        end = time.time()
        cpp_time = end - start
