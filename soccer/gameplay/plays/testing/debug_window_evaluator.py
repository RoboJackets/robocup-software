import play
import behavior
import main
import robocup
import constants
import time
import math

# This isn't a real play, but it's pretty useful
# Turn it on and we'll draw the window evaluator stuff on-screen from the ball to our goal
class DebugWindowEvaluator(play.Play):

    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')


    def execute_running(self):
        win_eval = robocup.WindowEvaluator(main.system_state())
        win_eval.debug = True
        windows, best = win_eval.eval_pt_to_our_goal(main.ball().pos)
