import main
import play
import behavior
import evaluation.field
import evaluation.passing_positioning
import evaluation.shooting
import evaluation.defensive_positioning
import visualization.overlay
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

    def __init__(self):
        super().__init__(continuous=True)
        self.add_state(TestAdaptiveFormationWeights.State.testPointCoeff,
                       behavior.Behavior.State.running)

        # Enable which portion we want to test
        self.add_transition(behavior.Behavior.State.start,
                            TestAdaptiveFormationWeights.State.testPointCoeff,
                            lambda: True, 'immediately')

    def execute_testPointCoeff(self):

        points = visualization.overlay.get_visualization_points(40, 80)
        vals = []

        for col in points:
            sublist = []
            for pt in col:
                # Uncomment which function we want graphed

                sublist.append(evaluation.field.field_pos_coeff_at_pos(
                    pt, 0.1, .2, 0.02))
                #sublist.append(1-evaluation.field.space_coeff_at_pos(pt))
                #sublist.append(evaluation.shooting.eval_shot(pt))
                #sublist.append(evaluation.defensive_positioning.estimate_risk_score(pt))
                #sublist.append(5 * evaluation.passing_positioning.eval_single_point(
                #    main.ball().pos, main.our_robots(), (0.01, 3, 0.02),
                #    (2, 2, 15, 1), pt.x, pt.y))

            vals.append(sublist)

        visualization.overlay.display_visualization_points(vals, True)
