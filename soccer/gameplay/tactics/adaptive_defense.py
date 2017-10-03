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

class AdaptiveDefense(composite_behavior.CompositeBehavior):

    # Weights for robot risk scores
    ROBOT_RISK_WEIGHTS = [1, 1, 3, 1, 1, 1]

    # Weights for the area risk scores
    AREA_RISK_WEIGHTS = [1, 3, 3, 1]
    class State(enum.Enum):
        # Basic blocking for right now
        # TODO: Add clearing mode
        defending = 1

    # defender_prioirities should have a length of 5 for all non-goalie robots
    def __init__(self, defender_priorities=[20,19,18,17,16]):
        super().__init__(continuous=True)

        if len(defender_priorities) != 5:
            raise RuntimeError(
                "defender_priorities should have a length of two")

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
        self.robot_risks = [] # List of tuples of robot risk scores, best shot pt, robot obj
        self.area_risk = [] # List of tuples of area risk score, (pos, scores) tuple, and robot obj
        self.agressiviness = 0 # Changes how to weight the robots towards higher risk opp

        self.uncertainty_coeff = 0.5
        self.future_times = [0.1, 0.5, 1]

        self.kick_eval.excluded_robots.clear()

        for bot in main.our_robots():
            self.kick_eval.add_excluded_robot(bot)

    def execute_running(self):
        # Calculate scores 
        # TODO: See if we can cache anything
        self.calculate_risks_scores()
        # Apply roles
        self.apply_blocking_roles()

    def calculate_risks_scores(self):
        # Get all opp robot scores
        self.calculate_robot_risk_scores()
        # Get predicted area scores
        self.calculate_area_risk_scores()
        # Merge predicted area and robot when they are close
        self.clean_risk_scores_lists()

    def calculate_robot_risk_scores(self):
        del self.robot_risks[:]

        max_dist = robocup.Point(constants.Field.Length, constants.Field.Width).mag()
        for bot in main.their_robots():
            if bot.visible:
                # Various sensitiviities to change how the distribution looks
                # Mostly shifts and scales from [0, 1] to [x, 1]
                dist_sens = 0.75
                angle_sens = 0.5
                ball_opp_sens = 1.5
                ball_goal_sens = 2.5

                # Distance to the ball
                ball_dist = pow(1 - dist_sens*(bot.pos - main.ball().pos).mag() / max_dist, 2)
                # Angle on goal, 1 when down the center line, 0 when in corner
                angle_to_goal = 1 - angle_sens*math.fabs(math.atan2(bot.pos.x, bot.pos.y) / (math.pi / 2))
                # Shot chance / Pass chance
                shot_pt, shot_chance = self.kick_eval.eval_pt_to_our_goal(bot.pos)
                pass_pt, pass_chance = self.kick_eval.eval_pt_to_robot(main.ball().pos, bot.pos)
                # Angle between ball-opponent-goal
                # Pseudo-chance to score on a one touch situation
                ball_opp_goal = 1 - math.pow((math.fabs(self.angle_between(main.ball().pos - bot.pos, bot.pos - shot_pt)) / math.pi), ball_opp_sens)
                # Angle between ball-goal-opponent
                # Distance the defense will need to shift to defend
                ball_goal_opp = 1 - math.pow(math.fabs(self.angle_between(main.ball().pos - shot_pt, shot_pt - bot.pos)) / math.pi, ball_goal_sens)

                risk_score = AdaptiveDefense.ROBOT_RISK_WEIGHTS[0] * ball_dist + \
                             AdaptiveDefense.ROBOT_RISK_WEIGHTS[1] * angle_to_goal + \
                             AdaptiveDefense.ROBOT_RISK_WEIGHTS[2] * shot_chance + \
                             AdaptiveDefense.ROBOT_RISK_WEIGHTS[3] * pass_chance + \
                             AdaptiveDefense.ROBOT_RISK_WEIGHTS[4] * ball_opp_goal + \
                             AdaptiveDefense.ROBOT_RISK_WEIGHTS[5] * ball_goal_opp

                risk_score /= sum(AdaptiveDefense.ROBOT_RISK_WEIGHTS)

                self.robot_risks.append((risk_score, shot_pt, bot))

                if self.debug:
                    main.system_state().draw_text("Risk: " + str(int(risk_score*100)), bot.pos, constants.Colors.White, "Defense: Risk")

    def calculate_area_risk_scores(self):
        # Areas is list of predictions, with an uncertainty variable
        # Think hurricane prediction cones

        del self.area_risk[:]
        
        for bot in main.their_robots():
            if bot.visible:
                # List of future positions based upon velocity and time
                future_pos = []
                # List of scores corresponding to those future positions
                future_scores = []

                for t in self.future_times:
                    # TODO: Think about bending line towards goal
                    future_pos.append(bot.pos + bot.vel*t)

                for idx, pos in enumerate(future_pos):
                    # Various sensitiviities to change how the distribution looks
                    # Mostly shifts and scales from [0, 1] to [x, 1]
                    sensitivity = 4
                    ball_goal_sens = 2.5

                    # How close they are to other robots on their team
                    opp_space = 1 - evaluation.field.space_coeff_at_pos(pos, [bot], main.their_robots(), sensitivity)
                    # Shot chance
                    shot_pt, shot_chance = self.kick_eval.eval_pt_to_our_goal(pos)
                    ball_goal_opp = 1 - math.pow(math.fabs(self.angle_between(main.ball().pos - shot_pt, shot_pt - pos)) / math.pi, ball_goal_sens)
                    field_pos = evaluation.field.field_pos_coeff_at_pos(pos, 0, 1, 0, False)

                    risk_score = AdaptiveDefense.AREA_RISK_WEIGHTS[0] * opp_space + \
                                 AdaptiveDefense.AREA_RISK_WEIGHTS[1] * shot_chance + \
                                 AdaptiveDefense.AREA_RISK_WEIGHTS[2] * ball_goal_opp + \
                                 AdaptiveDefense.AREA_RISK_WEIGHTS[3] * field_pos
                    risk_score /= sum(AdaptiveDefense.AREA_RISK_WEIGHTS)

                    future_scores.append(risk_score)

                    if self.debug:
                        main.system_state().draw_circle(pos, self.uncertainty_coeff*self.future_times[idx], constants.Colors.Red, "Defense: Areas")
                        main.system_state().draw_text("Risk: " + str(int(risk_score*100)), pos, constants.Colors.Red, "Defense: Areas")

                self.area_risk.append((max(future_scores), zip(future_pos, future_scores), bot))

    def clean_risk_scores_lists(self):
        # Removes any close duplicates between the areas and robots
        pass

    def apply_blocking_roles(self):
        # Get list of robots, sorted by risk
        # Apply blocking roles in order
        # If significant jump, maybe apply multiple robots?
        # Maybe just apply multiple robots for the highest risk
        # Spread thicker when fewere robots that are risky
        # Remove any robots past half (but don't remove their area)
        pass

    ## Gets the angle between two lines
    #  Angle is constrained to -pi to +pi
    def angle_between(self, line1, line2):
        c = line1.dot(line2) / line1.mag() / line2.mag()
        return math.acos(self.clip(c, -1, 1))

    ## Clips the val to be between the lower and upper boundries
    def clip(self, val, lower, upper):
        return min(max(val, lower), upper)