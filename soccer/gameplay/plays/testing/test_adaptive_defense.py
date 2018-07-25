import play
import behavior
import tactics.adaptive_defense
import visualization.overlay
import evaluation.field
import robocup
import main
import constants
import math


## Runs our Defense tactic
class TestAdaptiveDefense(play.Play):
    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            "immediately")

    def on_enter_running(self):
        b = tactics.adaptive_defense.AdaptiveDefense()
        self.add_subbehavior(b, name='defense', required=True)
        pass

    def execute_running(self):
        points = visualization.overlay.get_visualization_points(30, 45)
        vals = []

        for col in points:
            sublist = []
            for pt in col:
                # Uncomment which function we want graphed

                #sublist.append(self.robotPos(pt))
                #sublist.append(self.areaPos(pt))
                sublist.append(self.robotPos(pt) < self.areaPos(pt)*1.8)

                #max_min = max(self.areaPos(pt), self.robotPos(pt)) - min(self.areaPos(pt), self.robotPos(pt))
                #sublist.append(max_min*2)
                #sublist.append(evaluation.field.field_pos_coeff_at_pos(
                #    pt, 0.1, .2, 0.02))
                #sublist.append(1-evaluation.field.space_coeff_at_pos(pt))
                #sublist.append(evaluation.shooting.eval_shot(pt))
                #sublist.append(evaluation.defensive_positioning.estimate_risk_score(pt))
                #sublist.append(5 * evaluation.passing_positioning.eval_single_point(
                #    main.ball().pos, main.our_robots(), (0.01, 3, 0.02),
                #    (2, 2, 15, 1), pt.x, pt.y))

            vals.append(sublist)

        visualization.overlay.display_visualization_points(vals, True)

    def robotPos(self, pos):
        max_dist = robocup.Point(constants.Field.Length, constants.Field.Width).mag()
        our_goal = robocup.Point(0, 0)
        dist_sens = 1.5
        ball_opp_sens = 1.5

        ball_dist = pow(1 - dist_sens*(pos - main.ball().pos).mag() / max_dist, 2)
        ball_opp_goal = math.pow((math.fabs((main.ball().pos - pos).angle_between(pos - our_goal)) / math.pi), ball_opp_sens)

        weights = [1, 1]
        risk_score = weights[0] * ball_dist + \
                     weights[1] * ball_opp_goal

        risk_score /= sum(weights)

        return risk_score

    def areaPos(self, pos):
        max_dist = robocup.Point(constants.Field.Length, constants.Field.Width).mag()
        our_goal = robocup.Point(0, 0)
        ball_goal_sens = 2.5
        dist_sens = 1.5

        ball_dist = 1 - pow(1 - dist_sens*(pos - main.ball().pos).mag() / max_dist, 2)
        ball_goal_opp = 1 - math.pow(math.fabs((main.ball().pos - our_goal).angle_between(our_goal - pos)) / math.pi, ball_goal_sens)
        field_pos = evaluation.field.field_pos_coeff_at_pos(pos, 0, 1, 0, False)

        weights = [1, 2, 3]
        risk_score = weights[0] * ball_dist + \
                     weights[1] * ball_goal_opp + \
                     weights[2] * field_pos
        risk_score /= sum(weights)

        return risk_score

    def on_exit_running(self):
        self.remove_subbehavior('defense')
        pass

    @classmethod
    def handles_goalie(cls):
        return True