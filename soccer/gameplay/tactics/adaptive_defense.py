# Get risk of each opponent robot
# Get risk of areas (at a lower level compared to opponent robots)
#   Only apply risk to areas when a robot may be moving into that area
#   Defend area not robot when the robot is moving very quickly
#   Can use risk zones as prediction areas for likelyhood opp robots
#       move to that zone
# 
#
# How to map defenders onto offensive threats
# Always have 1 or 2 block direct shots
# Block direct shots from others
# Agressiviness tuner to choose how many robots
#   to bring to higher threat targets
#
# Always pull Max or N+1 defenders

import composite_behavior
import behavior
import constants
import robocup
import main
import enum
import math

import tactics.positions.submissive_goalie as submissive_goalie
import tactics.positions.submissive_defender as submissive_defender

import evaluation.field
import evaluation.linear_classification

class AdaptiveDefense(composite_behavior.CompositeBehavior):

    # Weights for robot risk scores
    # [ball_dist, ball_opp_goal]
    ROBOT_RISK_WEIGHTS = [1, 1]

    # Weights for the area risk scores
    # [ball_dist, ball_goal_opp, field_pos]
    AREA_RISK_WEIGHTS = [1, 2, 3]

    # Weights / Bias for whether a opponent is a forward or winger
    # Classifier returns true if it is a winger
    WING_FORWARD_WEIGHTS = [-1, 1.8]
    WING_FORWARD_BIAS    = 0
    WING_FORWARD_CUTOFF  = 0

    class State(enum.Enum):
        # Basic blocking for right now
        # TODO: Add clearing mode
        defending = 1

    # defender_prioirities should have a length of 5 for all non-goalie robots
    def __init__(self, defender_priorities=[20,19,18,17,16]):
        super().__init__(continuous=True)

        # TODO: Change this to number of robots - 1
        if len(defender_priorities) != 5:
            raise RuntimeError(
                "defender_priorities should have a length of five")

        self.add_state(AdaptiveDefense.State.defending,
                       behavior.Behavior.State.running)
        self.add_transition(behavior.Behavior.State.start,
                            AdaptiveDefense.State.defending,
                            lambda: True, "immediately")

        goalie = submissive_goalie.SubmissiveGoalie()
        goalie.shell_id = main.root_play().goalie_id
        self.add_subbehavior(goalie, "goalie", required=False)

        for num, priority in enumerate(defender_priorities):
            defender = submissive_defender.SubmissiveDefender()
            self.add_subbehavior(defender,
                                 'defender' + str(num + 1),
                                 required=False,
                                 priority=priority)

        self.debug = True
        self.kick_eval = robocup.KickEvaluator(main.system_state())
        self.num_of_defenders = 6 # TODO: Make variable
        self.robot_classes = [] # List of tuples of is_winger (!is_forward), class score, and robot obj
        self.agressiviness = 0 # Changes how to weight positioning on the wingers

        self.kick_eval.excluded_robots.clear()

        for bot in main.our_robots():
            self.kick_eval.add_excluded_robot(bot)

    def execute_running(self):
        # Classify the opponent robots as wingers or forwards
        self.classify_opponent_robots()
        # Apply roles
        self.apply_blocking_roles()

    def classify_opponent_robots(self):
        # Classify opponent robots as a winger or forward
        # Wingers are more towards the outside and require more space when defending
        # Forwards mostly have the ball or are near the ball and require shot blocking

        del self.robot_classes[:]

        for bot in main.their_robots():
            if bot.visible:
                robot_risk_score = self.calculate_robot_risk_scores(bot)
                area_risk_score  = self.calculate_area_risk_scores(bot)

                features = [robot_risk_score, area_risk_score]

                is_wing, class_score = evaluation.linear_classification.binary_classification(features,
                                            AdaptiveDefense.WING_FORWARD_WEIGHTS,
                                            AdaptiveDefense.WING_FORWARD_BIAS,
                                            AdaptiveDefense.WING_FORWARD_CUTOFF)
                
                self.robot_classes.append((is_wing, class_score, bot))

                if self.debug and is_wing:
                    main.system_state().draw_circle(bot.pos, 0.5, constants.Colors.White, "Defense: Class Wing")
                elif self.debug and not is_wing:
                    main.system_state().draw_circle(bot.pos, 0.5, constants.Colors.Black, "Defense: Class Forward")

       
    def calculate_robot_risk_scores(self, bot):
        max_dist = robocup.Point(constants.Field.Length, constants.Field.Width).mag()
        our_goal = robocup.Point(0, 0)
        dist_sens = 1.5
        ball_opp_sens = 1.5

        # How far away the robot is from the ball, closer is higher
        ball_dist = pow(1 - dist_sens*(bot.pos- main.ball().pos).mag() / max_dist, 2)
        # How large the angle is between the ball, opponent, and goal, smaller angle is better
        ball_opp_goal = math.pow((math.fabs((main.ball().pos - bot.pos).angle_between(bot.pos - our_goal)) / math.pi), ball_opp_sens)

        risk_score = AdaptiveDefense.ROBOT_RISK_WEIGHTS[0] * ball_dist + \
                     AdaptiveDefense.ROBOT_RISK_WEIGHTS[1] * ball_opp_goal

        risk_score /= sum(AdaptiveDefense.ROBOT_RISK_WEIGHTS)

        if self.debug:
            main.system_state().draw_text("Robot Risk: " + str(int(risk_score*100)), 
                bot.pos - robocup.Point(0, 0.25), constants.Colors.White, "Defense: Risk")
        
        return risk_score

    def calculate_area_risk_scores(self, bot):
        max_dist = robocup.Point(constants.Field.Length, constants.Field.Width).mag()
        our_goal = robocup.Point(0, 0)
        ball_goal_sens = 2.5
        dist_sens = 1.5

        # How far away the robot is from the ball, further is higher
        ball_dist = 1 - pow(1 - dist_sens*(bot.pos - main.ball().pos).mag() / max_dist, 2)
        # How large the angle is between the ball, goal, and opponent, smaller angle is better
        ball_goal_opp = 1 - math.pow(math.fabs((main.ball().pos - our_goal).angle_between(our_goal - bot.pos)) / math.pi, ball_goal_sens)
        # Location on the field based on closeness to the goal line, closer is better
        field_pos = evaluation.field.field_pos_coeff_at_pos(bot.pos, 0, 1, 0, False)

        risk_score = AdaptiveDefense.AREA_RISK_WEIGHTS[0] * ball_dist + \
                     AdaptiveDefense.AREA_RISK_WEIGHTS[1] * ball_goal_opp + \
                     AdaptiveDefense.AREA_RISK_WEIGHTS[2] * field_pos

        risk_score /= sum(AdaptiveDefense.AREA_RISK_WEIGHTS)

        if self.debug:
            main.system_state().draw_text("Area Risk: " + str(int(risk_score*100)), 
                bot.pos + robocup.Point(0, 0.25), constants.Colors.White, "Defense: Risk")

        return risk_score

    def build_defensive_formation(self):
        # Using the offsets and the classifier, create the formation
        # based on the number of offenders on each side of the ball
        # List the positions and what robot they are defending
        pass

    def apply_opponent_forces(self):
        # Apply opponent force fields to the positions in which we are defending
        # (Maybe apply all at a reduced level)
        # Treat XY axis as independent
        # Each is a spring following F=kd
        pass

    def apply_blocking_roles(self):
        # Apply in order of ball -> strikers -> wingers -> ball
        pass