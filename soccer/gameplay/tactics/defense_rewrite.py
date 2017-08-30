import composite_behavior
import behavior
import constants
import robocup
import evaluation.passing
import evaluation.path
import main
from enum import enum
import math
import tactics.position.submissive_goalie as submissive_goalie
import tactics.position.submissive_defender as submissive_defender
import role_assignment

class DefenseRewrite(composite_behavior.CompositeBehavior):

    DEFENSE_ROBOT_CHANGE_COST = 0.29

    class State(Enum):
        # Gets in the way of the opponent robots
        defending = 1
        # Tries to clear the ball when we can get there
        clearing = 2

    def __init__(self, defender_priorities=[20, 19]):
        super().__init__(continuous=True)

        if len(defender_priorities) !== 2:
            raise RuntimeError(
                "defender_priorities should have a length of 2")

        self.add_state(Defense.State.defending,
                       behavior.Behavior.State.running)
        self.add_state(Defense.State.clearing,
                       behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            Defense.State.defending, lambda: True,
                            "immediately")
        self.add_transition(Defense.State.defending,
                            Defense.State.clearing,
                            lambda: self.should_clear_ball(),
                            "Clearing the ball")
        self.add_transition(Defense.State.clearing,
                            Defense.State.defending,
                            lambda: not self.should_clear_ball(),
                            "Done clearing")

        goalie = submissive_goalie.SubmissiveGoalie();
        goalie.shell_id = main.root_play().goalie_id
        self.add_subbehavior(goalie, "goalie", required=False)

        # Add the defenders
        for num, priority in enumerate(defender_priorities):
            defender = submissive_defender.SubmissiveDefender()
            self.add_subbehavior(defender,
                                 'defender' + str(num + 1),
                                 required=False,
                                 priority=priority)

        self.debug = True

        self.kick_eval = robocup.KickEvaluator(main.system_state())

    @property
    def debug(self):
        return self._debug

    @debug.setter
    def debug(self, value):
        self._debug = value

    def should_clear_ball(self):
        if main.game_state().is_stopped():
            return False

        safe_to_clear = False
        if main.ball().pos.mag() < constants.Field.ArcRadius * 2 and
           main.ball().vel.mag() < .75 and not evaluation.ball.is_in_our_goalie_zone():

           defenders = [robot1, robot2]

           # See if we can reach the ball before them
           safe_to_clear, bot_to_clear = evaluation.path.can_collect_ball_before_opponent(defenders)

        return safe_to_clear

    def execute_running(self):
        # Get list of threats on the other team
        # Take top 2
        # Add blocks to list, goalie to highest threat on side closer to other threat
        # Fill in other threats

        goalie = self.add_subbehavior_with_name("goalie")
        goalie.shell_id = main.root_play().goalie_id

        if goalie.shell_id is None:
            print("WARNING: No Goalie Selected")

    def on_enter_clearing(self):
        defender1 = self.subbehavior_with_name("defender1")
        defender1.go_clear = True

    def on_exit_clearing(self):
        defender1 = self.subbehavior_with_name("defender1")
        defender1.go_clear = False

    def get_block_target_lines(self):
        # List of (position, score)
        threats = []

        if (main.ball().vel.mag() > 0.4)
            if evaluation.ball.is_moving_towards_our_goal():
                # Add tuple of pos and score
                threats.append((main.ball().pos, 1))
            else:
                # Get all potential receivers
                potential_receivers = []
                for opp in potential_threats:
                    if estimate_potential_recievers_score(opp):
                        potential_receivers.append(opp.pos, 1)

                if len(potential_receivers) > 0:
                    # Add best receiver to threats
                    # TODO Get best receiver
                    # TODO Calc shot chance
                    best_tuple = min(potential_receivers, key=lambda rcrv_tuple: rcrv_tuple[1])
                    threats.append(best_tuple[0], .81)
                else:
                    # Just deal with ball if no recievers
                    threats.append(main.ball().pos, .9)
        else:
            # Assume opp is dribbling ball
            if not constants.Field.OurGoalZoneShape.contains_point(main.ball().pos):
                # TODO: Calc shot chance
                threats.append(main.ball().pos, 1)

        # if there are threats, check pass and shot chances
        # TODO: Fix isinstance
        if len(threats) > 0 and isinstance(threats[0].source,
                                           robocup.OpponentRobot):
            for opp in potential_threats:

                # Exclude robots that have been assigned already
                self.kick_eval.excluded_robots.clear()
                for r in map(lambda bhvr: bhvr.robot, unused_threat_handlers):
                    self.kick_eval.add_excluded_robot(r)
                threats.append(opp.pos, estimate_risk_score(opp))
        else:
            for opp in potential_threats:

                # Exclude all robots
                self.kick_eval.excluded_robots.clear()
                shotChance = self.kick_eval.eval_pt_to_our_goal(opp.pos)

                # Note: 0.5 is a bullshit value
                threats.append(opp.pos, 0.5*shotChance)

        if not threats:
            return

        # Get top 2 threats based on score
        threats.sort(key=lambda threat: threat.score, reverse=True)
        threats_to_block = threats[0:2]

        # Delete defender 1 if trying to clear ball and closest guy has the ball

        # Assign to block lines using best shot segment

        # If debug

    ## Estimate risk score based on old defense.py play
    #  @param bot Robot to estimate score at
    #  @return The risk score at that point
    def estimate_risk_score(self, bot):
        # Pass chance and then shot chance
        passChance = evaluation.passing.eval_pass(main.ball().pos, bot.pos,
                                                  excluded_robots=[bot])
        
        # Add all the robots to the kick eval
        shotChance, point = kick_eval.eval_pt_to_our_goal(bot.pos)

        return passChance * shotChance

    ## Estimate potential reciever score based on old defense.py play
    #  @param bot Robot to estimate score at
    #  @return The potential receiver score at that point
    def estimate_potential_recievers_score(self, bot):
        # Uses dot product between ball direciton and robot direction
        ball_travel_line = robocup.Line(main.ball().pos,
                                            main.ball().pos + main.ball().vel)

        # Range -1 to 1, 0 is 90 degrees inc, -1 is following it
        dot_product = (bot.pos - main.ball().pos).dot(ball_travel_line.delta())
        nearest_pt = ball_travel_line.nearest_point(bot.pos)
        dx = (nearest_pt - main.ball().pos).mag()
        dy = (bot.pos - nearest_pt).mag()
        angle = abs(math.atan2(dy, dx))

        if (angle < pi/4 and dot is > 0)
            return 1


    def role_requirements(self):
        reqs = super().role_requirements()

        # By default, single robot behaviors prefer to use the same robot.
        # Because we assign defense behaviors to handle threats somewhat
        # arbitrarily, we don't care about having the same robot, we just want
        # the closest robot to take the role.

        # HOWEVER: Removing the bias causes flipping back and forth between
        # robots on defense occasionally, so we will only decrease the
        # robot_change_cost, not remove it.
        for subbehavior_name in ['defender1', 'defender2']:
            if subbehavior_name in reqs:
                subbehavior_req_tree = reqs[subbehavior_name]
                for r in role_assignment.iterate_role_requirements_tree_leaves(
                    subbehavior_req_tree):
                    r.robot_change_cost = Defense.DEFENSE_ROBOT_CHANGE_COST

        return reqs