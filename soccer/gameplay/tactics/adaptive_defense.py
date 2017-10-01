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

import evaulation.field

class AdaptiveDefense(composite_behavior.CompositeBehavior):
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

        self.kick_eval.excluded_robots.clear()

        for bot in main.our_robots():
            self.kick_eval.add_excluded_robot(bot)

    def execute_running(self):
        # Calculate scores (See if we can cache anything easily)
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
        pass

    def calculate_robot_risk_scores(self):
        # Big ones to take into account (Robot scores)
        # Dist to ball^2
        # Angle on goal (RL boxcar looking thing)
        # Shot chance
        # Pass chance

        # Ball-opponent-goal (one touch) (smaller is higher)
        # Ball-goal-opponent (def movment) (larger is higher)
        max_dist = robocup.Point(constants.Field.Length, constants.Field.Width).mag()
        for bot in main.their_robots():
            if bot.visible:
                ball_dist = pow(1 - (bot.pos - main.ball().pos).mag() / max_dist, 2)
                angle_to_goal = 1 - math.fabs(
                    math.atan2(bot.pos.x, bot.pos.y) / (math.pi / 2))
                shot_pt, shot_chance = self.kick_eval.eval_pt_to_our_goal(bot.pos)
                pass_pt, pass_chance = self.kick_eval.eval_pt_to_robot(main.ball().pos, bot.pos)
                #TODO: Think about using pass_pt instead of bot.pos
                ball_opp_goal = 1 - (math.fabs(self.angle_between(main.ball().pos - bot.pos, bot.pos - shot_pt)) / math.pi)
                ball_goal_opp = math.fabs(self.angle_between(main.ball().pos - shot_pt, bot.pos - shot_pt)) / math.pi

                # TODO: Fix weights
                weights = [1, 1, 1, 1, 1, 1]
                risk_score = weights[0] * ball_dist / max_dist + \
                             weights[1] * angle_to_goal + \
                             weights[2] * shot_chance*pass_chance + \
                             weights[3] * pass_chance + \
                             weights[4] * ball_opp_goal + \
                             weights[5] * ball_goal_opp

                risk_score /= sum(weights)

                self.robot_risks.append((risk_score, shot_pt, bot))

                if self.debug:
                    main.system_state().draw_text("Risk: " + str(int(risk_score*100)), bot.pos, constants.Colors.White, "Defense: Risk")

    def calculate_area_risk_scores(self):
        # Big ones to take into account (areas)
        # Only grab inc opponent traj
        # Centroid dist to corresponding robot (larger is higher)
        # Space
        # Shot chance
        # Pass to a few areas in front (not super important)

        # Areas is list of predictions, with an uncertainty variable
        # Think hurricane prediction cones
        areas = []
        uncertainty_coeff = 1
        future_times = [0.25, 0.5, 1, 2, 3]

        for bot in main.their_robots():
            if bot.visible:
                future_pos = []
                future_scores = []

                for t in future_times:
                    # TODO: Think about bending line towards goal
                    future_pos.append(bot.pos + bot.vel*t)

                for pos in future_pos:
                    # Space (both opp and our) (very sensitive)
                    # TODO: Do we actually need opp score
                    sensitivity = 15
                    our_space = evaluation.field.space_coeff_at_pos(pos, robots=main.their_robots(), sensitivity=sensitivity)
                    opp_space = evaluation.field.space_coeff_at_pos(pos, robots=main.our_robots(), sensitivity=sensitivity)
                    # TODO: Get rest of scores

                    weights = [1, 1]
                    risk_score = (weights[0] * (1 - our_space) + \
                                  weights[1] * opp_space)
                    risk_score /= sum(weights)

                    future_scores.append(risk_score)


                areas.append(zip(future_pos, future_scores))

                self.area_risk((sum(future_scores), areas, bot))


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