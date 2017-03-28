import main
import play
import behavior
import evaluation.field
import evaluation.passing_positioning
import evaluation.shooting
import evaluation.defensive_positioning
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
        mode = 1
        self.add_transition(behavior.Behavior.State.start,
                            TestAdaptiveFormationWeights.State.testPointCoeff, lambda: mode == 1,
                            'immediately')
        self.add_transition(behavior.Behavior.State.start,
                            TestAdaptiveFormationWeights.State.testBestPass, lambda: mode == 2,
                            'immediately')


    def on_enter_testPointCoeff(self):
        evaluation.defensive_positioning.find_defense_positions()

    def execute_testPointCoeff(self):
        # Number of boxes width and length wise
        num_width = 10
        num_length = 10

        val = 0
        max_val = 0
        max_x = 0
        max_y = 0

        # 1/2 the Width/Length of the boxes
        x_half = 0.5 * constants.Field.Width / num_width
        y_half = 0.5 * constants.Field.Length / num_length

        
        for x in range(-1*round(num_width/2), round(num_width/2)):
            for y in range(0, num_length):
                # X/Y for center of the boxes
                x_cent = x * constants.Field.Width / num_width + x_half
                y_cent = y * constants.Field.Length / num_length + y_half

                if constants.Field.TheirGoalZoneShape.contains_point(robocup.Point(x_cent, y_cent)):
                    continue

                # Uncomment which function we want graphed
                # 1: Field Pos Coeff
                # 2: Space Coeff
                # 3: Shot Eval %
                # 4: Ball Coeff
                # 5: Estimate Chance to Block Kick
                # 6: Risk Score
                # 

                #val = evaluation.field.field_pos_coeff_at_pos(robocup.Point(x_cent, y_cent), 0.1, .2, 0.02)
                #val = 1-evaluation.field.space_coeff_at_pos(robocup.Point(x_cent, y_cent))
                #val = evaluation.shooting.eval_shot(robocup.Point(x_cent, y_cent))
                #val = 1-evaluation.field.ball_coeff_at_pos(robocup.Point(x_cent, y_cent))
                #val = evaluation.defensive_positioning.estimate_kick_block_percent \
                #    (robocup.Point(x_cent, y_cent), robocup.Point(0, 0), main.our_robots())
                #val = evaluation.defensive_positioning.estimate_risk_score(robocup.Point(x_cent, y_cent))
                #val = (1-evaluation.field.space_coeff_at_pos(robocup.Point(x_cent, y_cent), [], main.our_robots())) * \
                #        evaluation.defensive_positioning.estimate_risk_score(robocup.Point(x_cent, y_cent))

                kick_eval = robocup.KickEvaluator(main.system_state())
                for bot in main.our_robots():
                    kick_eval.add_excluded_robot(bot)
                #val = kick_eval.eval_pt_to_pt(robocup.Point(x_cent, y_cent), robocup.Point(0, 3), 0.1)

                # Find max
                if (val > max_val):
                    max_val = val
                    max_x = x_cent
                    max_y = y_cent

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
                main.system_state().draw_polygon(rect, val_color, "Density")

        self.special_point = evaluation.defensive_positioning.create_area_defense_zones()
        x_cent = self.special_point.x
        y_cent = self.special_point.y

        rect = [robocup.Point(x_cent-x_half, y_cent-y_half),
                robocup.Point(x_cent+x_half, y_cent-y_half),
                robocup.Point(x_cent+x_half, y_cent+y_half),
                robocup.Point(x_cent-x_half, y_cent+y_half)]
        # Make a white rect at the max value
        val_color = (255, 255, 255)

        # Draw onto the Debug layer
        main.system_state().draw_polygon(rect, val_color, "Max")

    def execute_testBestPass(self):
        # Use as a way to test the pass weights for the best pass position
        evaluation.passing_positioning.eval_best_receive_point(main.ball().pos, None, [], (0.1, .2, 0.02), (2, 10, 0, 10), True)
