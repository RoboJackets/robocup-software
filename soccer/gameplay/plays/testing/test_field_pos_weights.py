import main
import play
import behavior
import evaluation.field
import constants
import robocup
import math
import enum

class TestFieldPosWeights(play.Play):
    class State(enum.Enum):
        test=1

    def __init__(self):
        super().__init__(continuous=True)

        self.add_state(TestFieldPosWeights.State.test, behavior.Behavior.State.running)
        self.add_transition(behavior.Behavior.State.start,
                            TestFieldPosWeights.State.test, lambda: True,
                            'immediately')

        self.win_eval = robocup.WindowEvaluator(main.system_state())
        self._debug = True

    @property
    def debug(self):
        return self._debug

    @debug.setter
    def debug(self, value):
        self._debug = value

    def execute_test(self):
        # Based on ball pos
        #print(evaluation.field.field_pos_coeff_at_pos(main.ball().pos, 0.2, 1, 1))
        num_width = 40
        num_length = 50

        for x in range(-1*round(num_width/2), round(num_width/2)):
            for y in range(0, num_length):
                x_half = 0.5 * constants.Field.Width / num_width
                y_half = 0.5 * constants.Field.Length / num_length
                x_cent = x * constants.Field.Width / num_width + x_half
                y_cent = y * constants.Field.Length / num_length + y_half
                #val = evaluation.field.field_pos_coeff_at_pos(robocup.Point(x_cent, y_cent), 0.2, 1, 1)
                val = evaluation.field.space_coeff_at_pos(robocup.Point(x_cent, y_cent))

                val = min(val, 1)

                rect = [robocup.Point(x_cent-x_half, y_cent-y_half),
                        robocup.Point(x_cent+x_half, y_cent-y_half),
                        robocup.Point(x_cent+x_half, y_cent+y_half),
                        robocup.Point(x_cent-x_half, y_cent+y_half)]
                val_color = (round(val*255), 0, round((1-val)*255))

                main.system_state().draw_polygon(rect, val_color, "Debug")

    def on_exit_test(self):
        print("left")