import play
import behavior
import main
import robocup
import evaluation
import constants
import time

# This isn't a real play, but it's pretty useful
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

        win_eval = evaluation.window_evaluator.WindowEvaluator()
        win_eval.debug = True
        start = time.time()
        windows,best = win_eval.eval_pt_to_seg(pt, seg)
        end = time.time()
        py_time = end - start

        win_eval_cpp = robocup.WindowEvaluator(main.system_state())
        win_eval_cpp.debug = True
        start = time.time()
        windows,best = win_eval_cpp.eval_pt_to_seg(pt, seg)
        end = time.time()
        cpp_time = end - start

        if self.avg_count < 100:
            self.avg_time_saved += py_time - cpp_time
            self.avg_count += 1
        else:
            print(self.avg_time_saved / self.avg_count)