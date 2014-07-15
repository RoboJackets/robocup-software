import play
import behavior
import main
import robocup
import evaluation
import constants


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
        pt = main.ball().pos
        seg = constants.Field.OurGoalSegment

        win_eval = evaluation.window_evaluator.WindowEvaluator()
        windows, best = win_eval.eval_pt_to_seg(pt, seg)

        main.system_state().draw_circle(pt, 0.03, constants.Colors.Blue, "DebugWindows")
        main.system_state().draw_line(seg, constants.Colors.Blue, "DebugWindows")
        if best != None:
            main.system_state().draw_line(best.segment, constants.Colors.Green, "DebugWindows")
            main.system_state().draw_line(robocup.Line(pt, best.segment.center()), constants.Colors.Green, "DebugWindows")
