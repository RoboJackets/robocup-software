import main
import play
import behavior
import evaluation.field
import evaluation.passing
import constants
import robocup
import math
import enum

class TestAdaptiveFormationWeights(play.Play):
    class State(enum.Enum):
        # Draws 2D probabilty plot onto the field
        # Red is higher probability
        # Blue is lower
        testPointCoeff = 1

        # Plots all the lines from bestPass
        # Red is higher score
        testBestPass = 2

    def __init__(self):
        super().__init__(continuous=True)
        self.add_state(TestAdaptiveFormationWeights.State.testPointCoeff, behavior.Behavior.State.running)
        self.add_state(TestAdaptiveFormationWeights.State.testBestPass, behavior.Behavior.State.running)

        # Enable which portion we want to test
        self.add_transition(behavior.Behavior.State.start,
                            TestAdaptiveFormationWeights.State.testPointCoeff, lambda: False,
                            'immediately')
        self.add_transition(behavior.Behavior.State.start,
                            TestAdaptiveFormationWeights.State.testBestPass, lambda: True,
                            'immediately')


    def execute_testPointCoeff(self):
        # Number of boxes width and length wise
        num_width = 40
        num_length = 50

        for x in range(-1*round(num_width/2), round(num_width/2)):
            for y in range(0, num_length):
                # 1/2 the Width/Length of the boxes
                x_half = 0.5 * constants.Field.Width / num_width
                y_half = 0.5 * constants.Field.Length / num_length
                # X/Y for center of the boxes
                x_cent = x * constants.Field.Width / num_width + x_half
                y_cent = y * constants.Field.Length / num_length + y_half
                
                # Uncomment which function we want graphed
                #val = evaluation.field.field_pos_coeff_at_pos(robocup.Point(x_cent, y_cent), 0.2, 1, 1)
                val = evaluation.field.space_coeff_at_pos(robocup.Point(x_cent, y_cent))

                # Force between 0 and 1
                val = min(val, 1)
                val = max(val, 0)

                rect = [robocup.Point(x_cent-x_half, y_cent-y_half),
                        robocup.Point(x_cent+x_half, y_cent-y_half),
                        robocup.Point(x_cent+x_half, y_cent+y_half),
                        robocup.Point(x_cent-x_half, y_cent+y_half)]
                # Linear interpolation between Red and Blue
                val_color = (round(val*255), 0, round((1-val)*255))

                # Draw onto the Debug layer
                main.system_state().draw_polygon(rect, val_color, "Debug")

    def execute_testBestPass(self):
        evaluation.passing.eval_best_receive_point(main.ball().pos, None, [], True)