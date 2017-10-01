import play
import behavior
import tactics.adaptive_defense
import visualization.overlay
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

        self.kick_eval = robocup.KickEvaluator(main.system_state())
        self.kick_eval.excluded_robots.clear()

        for bot in main.our_robots():
            self.kick_eval.add_excluded_robot(bot)

    def on_enter_running(self):
        #b = tactics.adaptive_defense.AdaptiveDefense()
        #self.add_subbehavior(b, name='defense', required=True)
        pass

    def execute_running(self):
        points = visualization.overlay.get_visualization_points(10, 20)
        vals = []

        for col in points:
            sublist = []
            for pt in col:
                # Uncomment which function we want graphed

                sublist.append(self.test(pt))
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

    def test(self, pos):
        max_dist = robocup.Point(constants.Field.Length, constants.Field.Width).mag()

        dist_sens = 0.75
        angle_sens = 0.5
        ball_opp_sens = 1.5
        ball_goal_sens = 2.5
        ball_dist = pow(1 - dist_sens*(pos - main.ball().pos).mag() / max_dist, 2)
        angle_to_goal = 1 - angle_sens*math.fabs(math.atan2(pos.x, pos.y) / (math.pi / 2))
        shot_pt, shot_chance = self.kick_eval.eval_pt_to_our_goal(pos)
        pass_pt, pass_chance = self.kick_eval.eval_pt_to_robot(main.ball().pos, pos)
        #TODO: Think about using pass_pt instead of bot.pos
        ball_opp_goal = 1 - math.pow((math.fabs(self.angle_between(main.ball().pos - pos, pos - shot_pt)) / math.pi), ball_opp_sens)
        ball_goal_opp = 1 - math.pow(math.fabs(self.angle_between(main.ball().pos - shot_pt, shot_pt - pos)) / math.pi, ball_goal_sens)

        # TODO: Fix weights
        weights = [5, 0, 0, 0, 2, 2] # [5, 1, 1, 1, 2, 2]
        risk_score = weights[0] * ball_dist + \
                     weights[1] * angle_to_goal + \
                     weights[2] * shot_chance*pass_chance + \
                     weights[3] * pass_chance + \
                     weights[4] * ball_opp_goal + \
                     weights[5] * ball_goal_opp

        risk_score /= sum(weights)

        return risk_score

    def on_exit_running(self):
        #self.remove_subbehavior('defense')
        pass

    @classmethod
    def handles_goalie(cls):
        return True


    ## Gets the angle between two lines
    #  Angle is constrained to -pi to +pi
    def angle_between(self, line1, line2):
        c = line1.dot(line2) / line1.mag() / line2.mag()
        return math.acos(self.clip(c, -1, 1))

    ## Clips the val to be between the lower and upper boundries
    def clip(self, val, lower, upper):
        return min(max(val, lower), upper)