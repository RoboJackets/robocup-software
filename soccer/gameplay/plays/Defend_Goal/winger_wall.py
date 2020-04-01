import composite_behavior
import behavior
import constants
import robocup
import standard_play
import evaluation.passing
import evaluation.defensive_positioning
import evaluation.path
import main
import math
import role_assignment
from enum import Enum
import evaluation.field
import evaluation.linear_classification

import tactics.positions.submissive_goalie as submissive_goalie
import tactics.positions.wing_defender as wing_defender
import tactics.wall as wall
import situational_play_selection

## Defense play that utilizes a wall and wingers
class WingerWall(standard_play.StandardPlay):


    _situationList = [
        situational_play_selection.SituationalPlaySelector.Situation.DEFEND_GOAL,
        situational_play_selection.SituationalPlaySelector.Situation.DEFENSIVE_PILEUP
    ] # yapf: disable



    # Weights for robot risk scores
    # [ball_dist, ball_opp_goal]
    ROBOT_RISK_WEIGHTS = [1, 1]

    # Weights for the area risk scores
    # [ball_dist, ball_goal_opp, field_pos]
    AREA_RISK_WEIGHTS = [1, 2, 3]

    # Weights / Bias for whether a opponent is a forward or winger
    # Classifier returns true if it is a winger
    WING_FORWARD_WEIGHTS = [-1.1, 1.8]
    WING_FORWARD_BIAS = 0
    WING_FORWARD_CUTOFF = 0

    class State(Enum):
        defending = 0

    def __init__(self):
        super().__init__(continuous=True)

        goalie = submissive_goalie.SubmissiveGoalie()
        goalie.shell_id = main.root_play().goalie_id
        self.add_subbehavior(goalie, "goalie", required=True)

        self.add_state(WingerWall.State.defending,
                       behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            WingerWall.State.defending, lambda: True,
                            "immediately")

        self.aggression = 1
        self.kick_eval = robocup.KickEvaluator(main.system_state())
        self.wingers = []
        self.forwards = []
        self.max_wingers = 3
        self.max_wall = 3
        self.assigned_wingers = 0
        # List of tuples of (class score, robot obj)

        self.kick_eval.excluded_robots.clear()

        for bot in main.our_robots():
            self.kick_eval.add_excluded_robot(bot)

    def execute_defending(self):
        # Classify the opponent robots as wingers or forwards
        self.classify_opponent_robots()
        # Apply roles
        self.apply_blocking_roles()

    ## Stop winger wall from using adding standard defense
    def use_standard_defense(self):
        pass

    ## Classify opponent robots as a 'winger' or 'forward'
    #
    # Wingers are positioned away from the ball and may be passed to
    # Forwards mostly have the ball or are near the ball and require direct shot blocking
    def classify_opponent_robots(self):
        del self.wingers[:]
        del self.forwards[:]

        for bot in main.their_robots():
            if bot.visible and bot.pos.y < constants.Field.Length / 2:
                robot_risk_score = self.calculate_robot_risk_scores(bot)
                area_risk_score = self.calculate_area_risk_scores(bot)

                features = [robot_risk_score, area_risk_score]
                is_wing, class_score = evaluation.linear_classification.binary_classification(
                    features, WingerWall.WING_FORWARD_WEIGHTS,
                    WingerWall.WING_FORWARD_BIAS,
                    WingerWall.WING_FORWARD_CUTOFF)

                is_wing = not is_wing  #appears tobe inverted fix TODO

                if is_wing:
                    self.wingers.append((class_score, bot))
                else:
                    self.forwards.append((class_score, bot))

    ## Set up wingers and walls
    #
    # calls setup wing defenders and setup wall
    def apply_blocking_roles(self):
        self._setup_wing_defenders()
        wall_defenders = max(3, len(main.our_robots()) - self.assigned_wingers)
        self._setup_wall(wall_defenders)

    def _setup_submissive_defenders(self, number):
        # Last priority defender not yet implemented
        pass

    ## Sets up our wing defenders
    #
    # After we classify the enemy wingers, we assign our own wingers to them
    def _setup_wing_defenders(self):
        self.wingers = sorted(
            self.wingers, key=lambda winger: winger[0], reverse=True)
        current_wingers = len(self.wingers)
        for i in range(self.max_wingers):
            name = 'winger_skill' + str(i)
            if current_wingers > i:
                rob = self.wingers[i][1]
                score = self.wingers[i][0]
                # Assign an opposing winger to a wing defender
                if self.assigned_wingers <= i:
                    # Assign to a new wing defender
                    defender = wing_defender.WingDefender(
                        mark_robot=rob,
                        goalside_ratio=self._calc_depth_ratio(rob),
                        distance=self._calc_wing_distance(rob, score))
                    self.add_subbehavior(defender, name)
                else:
                    # Assign to an existing wing defender
                    self.subbehavior_with_name(name).mark_robot = rob
                    self.subbehavior_with_name(
                        name).goalside_ratio = self._calc_depth_ratio(rob)
                    self.subbehavior_with_name(
                        name).distance = self._calc_wing_distance(rob, score)

            elif self.assigned_wingers > i and self.assigned_wingers > 0:
                # Remove extra wing defender
                self.remove_subbehavior(name)

        self.assigned_wingers = min(self.max_wingers, current_wingers)

    ## Set up the wall defenders
    #
    # After we classify the enemy forward attacker,
    # we will build a wall to block their shot
    def _setup_wall(self, wall_defenders=3):
        self.forwards = sorted(self.forwards, key=lambda winger: winger[0])
        if self.has_subbehavior_with_name('form wall'):
            self.subbehavior_with_name(
                'form wall').mark_point = main.ball().pos
            self.subbehavior_with_name(
                'form wall').num_defenders = wall_defenders
        else:
            tact = wall.Wall(
                mark_point=main.ball().pos, num_defenders=wall_defenders)
            self.add_subbehavior(tact, "form wall")

    ## Factor for how close the robot is to our goal
    #
    # increases as robot gets closer to our goal
    def _calc_depth_ratio(self, opp_robot):
        return min(1, (1 - opp_robot.pos.y / (constants.Field.Length / 2)) *
                   self.aggression)

    ## Factors how close the robot is to the right side of the field?
    #
    # Increases as the robot position x increases.
    def _calc_wing_distance(self, opp_robot, score):
        return (constants.Robot.Radius + abs(opp_robot.pos.x) /
                (constants.Field.Width / 2)) / (self.aggression)

    ## Calculates robot risk using ball dist to goal and angle
    def calculate_robot_risk_scores(self, bot):
        max_dist = robocup.Point(constants.Field.Length,
                                 constants.Field.Width).mag()
        our_goal = robocup.Point(0, 0)
        dist_sens = 1.5
        ball_opp_sens = 1.5

        # How far away the robot is from the ball, closer is higher
        ball_dist = pow(1 - dist_sens *
                        (bot.pos - main.ball().pos).mag() / max_dist, 2)
        # How large the angle is between the ball, opponent, and goal, smaller angle is better
        ball_opp_goal = math.pow((math.fabs(
            (main.ball().pos - bot.pos).angle_between(bot.pos - our_goal)) /
                                  math.pi), ball_opp_sens)

        risk_score = WingerWall.ROBOT_RISK_WEIGHTS[0] * ball_dist + \
                     WingerWall.ROBOT_RISK_WEIGHTS[1] * ball_opp_goal

        risk_score /= sum(WingerWall.ROBOT_RISK_WEIGHTS)

        return risk_score

    ## Calculates area risk based on angle and distance
    def calculate_area_risk_scores(self, bot):
        max_dist = robocup.Point(constants.Field.Length,
                                 constants.Field.Width).mag()
        our_goal = robocup.Point(0, 0)
        ball_goal_sens = 2.5
        dist_sens = 1.5

        # How far away the robot is from the ball, further is higher
        ball_dist = 1 - pow(1 - dist_sens *
                            (bot.pos - main.ball().pos).mag() / max_dist, 2)
        # How large the angle is between the ball, goal, and opponent, smaller angle is better
        ball_goal_opp = 1 - math.pow(
            math.fabs(
                (main.ball().pos - our_goal).angle_between(our_goal - bot.pos))
            / math.pi, ball_goal_sens)
        # Location on the field based on closeness to the goal line, closer is better
        field_pos = evaluation.field.field_pos_coeff_at_pos(bot.pos, 0, 1, 0,
                                                            False)

        risk_score = WingerWall.AREA_RISK_WEIGHTS[0] * ball_dist + \
                     WingerWall.AREA_RISK_WEIGHTS[1] * ball_goal_opp + \
                     WingerWall.AREA_RISK_WEIGHTS[2] * field_pos

        risk_score /= sum(WingerWall.AREA_RISK_WEIGHTS)

        return risk_score
