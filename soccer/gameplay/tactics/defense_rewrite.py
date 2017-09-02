import composite_behavior
import behavior
import constants
import robocup
import evaluation.passing
import evaluation.path
import main
from enum import Enum
import math
import tactics.positions.submissive_goalie as submissive_goalie
import tactics.positions.submissive_defender as submissive_defender
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

        if len(defender_priorities) != 2:
            raise RuntimeError(
                "defender_priorities should have a length of 2")

        self.add_state(DefenseRewrite.State.defending,
                       behavior.Behavior.State.running)
        self.add_state(DefenseRewrite.State.clearing,
                       behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            DefenseRewrite.State.defending, lambda: True,
                            "immediately")
        self.add_transition(DefenseRewrite.State.defending,
                            DefenseRewrite.State.clearing,
                            lambda: self.should_clear_ball(),
                            "Clearing the ball")
        self.add_transition(DefenseRewrite.State.clearing,
                            DefenseRewrite.State.defending,
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
        if (main.ball().pos.mag() < constants.Field.ArcRadius * 2 and
           main.ball().vel.mag() < .75 and not evaluation.ball.is_in_our_goalie_zone()):

            defender1 = self.subbehavior_with_name('defender1')
            defender2 = self.subbehavior_with_name('defender2')
            if (defender1.robot != None and defender2.robot != None):

                defenders = [defender1, defender2]

                # See if we can reach the ball before them
                # Joe: Double check this method
                #safe_to_clear, bot_to_clear = evaluation.path.can_collect_ball_before_opponent(defenders)
                safe_to_clear = False

        return safe_to_clear

    def execute_running(self):
        goalie = self.subbehavior_with_name("goalie")
        goalie.shell_id = main.root_play().goalie_id

        if goalie.shell_id is None:
            print("WARNING: No Goalie Selected")

        self.find_and_set_defender_location()

    def on_enter_clearing(self):
        defender1 = self.subbehavior_with_name("defender1")
        defender1.go_clear = True

    def on_exit_clearing(self):
        defender1 = self.subbehavior_with_name("defender1")
        defender1.go_clear = False

    def find_and_set_defender_location(self):
        goalie = self.subbehavior_with_name('goalie')
        defender1 = self.subbehavior_with_name('defender1')
        defender2 = self.subbehavior_with_name('defender2')
        unused_threat_handlers = list(filter(
            lambda bhvr: bhvr.robot is not None, [goalie, defender1, defender2]))

        threats = self.get_threat_list(unused_threat_handlers)

        if not threats:
            return

        # Get top 2 threats based on score
        threats.sort(key=lambda threat: threat[1], reverse=True)
        threats_to_block = threats[0:2]
        assigned_handlers = [[], []]

        # If we clearing the ball, assign the clearer to the most important
        # threat (the ball). This prevents assigning the non-clearing robot
        # to mark the ball and causing crowding.
        if (defender1.state == submissive_defender.SubmissiveDefender.State.clearing):
            if defender1 in unused_threat_handlers:
                if (threats_to_block[0][0].dist_to(main.ball().pos) < constants.Robot.Radius * 2):
                    defender_idx = unused_threat_handlers.index(defender1)
                    assigned_handlers[0].append(unused_threat_handlers[defender_idx])
                    del unused_threat_handlers[defender_idx]

        
        self.assign_handlers_to_threats(assigned_handlers, unused_threat_handlers, threats_to_block)

        self.set_defender_block_lines(threats_to_block, assigned_handlers)
        # If debug

    ## Gets list of threats
    #  @return tuple of threat positions and score (unordered)
    def get_threat_list(self, unused_threat_handlers):
        # List of (position, score)
        threats = []
        potential_threats = main.their_robots()

        # find the primary threat
        # if the ball is not moving OR it's moving towards our goal, it's the primary threat
        # if it's moving, but not towards our goal, the primary threat is the robot on their team most likely to catch it
        # Joe: This can be improved a bunch
        if (main.ball().vel.mag() > 0.4):
            if evaluation.ball.is_moving_towards_our_goal():
                # Add tuple of pos and score
                threats.append((main.ball().pos, 1))
            else:
                # Get all potential receivers
                potential_receivers = []
                for opp in potential_threats:
                    if self.estimate_potential_recievers_score(opp):
                        potential_receivers.append((opp.pos, 1))

                if len(potential_receivers) > 0:
                    # Add best receiver to threats
                    # TODO Calc shot chance
                    best_tuple = min(potential_receivers, key=lambda rcrv_tuple: rcrv_tuple[1])
                    threats.append((best_tuple[0], .81))
                else:
                    # Just deal with ball if no recievers
                    threats.append((main.ball().pos, .9))
        else:
            # Assume opp is dribbling ball
            if not constants.Field.OurGoalZoneShape.contains_point(main.ball().pos):
                # TODO: Calc shot chance
                threats.append((main.ball().pos, 1))



        # if there are threats, check pass and shot chances
        # If the first item is not a ball, it is most likely a pass
        if len(threats) > 0 and threats[0][0] != main.ball().pos:
            for opp in potential_threats:

                # Exclude robots that have been assigned already
                # Joe: These should be all robots in list, none are assigned
                self.kick_eval.excluded_robots.clear()
                for r in map(lambda bhvr: bhvr.robot, unused_threat_handlers):
                    self.kick_eval.add_excluded_robot(r)

                threats.append((opp.pos, self.estimate_risk_score(opp)))
        else:
            for opp in potential_threats:

                # Exclude all robots
                self.kick_eval.excluded_robots.clear()
                for r in main.our_robots():
                    self.kick_Eval.add_excluded_robot(r)

                shotChance = self.kick_eval.eval_pt_to_our_goal(opp.pos)

                # Note: 0.5 is a bullshit value
                threats.append((opp.pos, 0.5*shotChance))

        return threats

    ## Estimate risk score based on old defense.py play
    #  @param bot Robot to estimate score at
    #  @return The risk score at that point
    def estimate_risk_score(self, bot):
        # Pass chance and then shot chance
        passChance = evaluation.passing.eval_pass(main.ball().pos, bot.pos,
                                                  excluded_robots=[bot])
        
        # Add all the robots to the kick eval
        point, shotChance = self.kick_eval.eval_pt_to_our_goal(bot.pos)

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

        # Joe: This is probs where most of the error is coming from
        if (angle < pi/4 and dot > 0):
            return 1
        else:
            return 0

    ## Assigns the defenders to threats
    #  @param assigned_handlers List of list, [ a ... ] where a represents a list of defenders assigned to threat a
    #  @param unused_threat_handlers List of defenders that are unused currently
    #  @param threats_to_block List of threats that we have to deal with, tuple with position and threat score
    def assign_handlers_to_threats(self, assigned_handlers, unused_threat_handlers, threats_to_block):
        threat_idx = 0
        while len(unused_threat_handlers) > 0:
            assigned_handlers[threat_idx].append(unused_threat_handlers[0])
            del unused_threat_handlers[0]

            threat_idx = (threat_idx + 1) % len(threats_to_block)


    ## Assigns the locations for each robot to block given a threat and list of robots to block each threat
    #  @param threats_to_block List of threats that we have to deal with, tuple with position and threat score 
    #  @param assigned_handlers List of list, [ a ... ] where a represents a list of defenders assigned to threat a
    def set_defender_block_lines(self, threats_to_block, assigned_handlers):
        goalie = self.subbehavior_with_name('goalie')

        # For each threat
        # threats_to_block (list of threats to block and their threat score)
        # assigned_handlers (list of defenders to assigned to threats)
        #
        # If more than 1, put goalie in the middle
        # Get best shot line
        # 
        # For each handler, find the angular width that each can defend
        # 
        # Get best point in list and set
        for threat_idx in range(len(threats_to_block)):
            threat = threats_to_block[threat_idx]
            assigned_handler = assigned_handlers[threat_idx]

            # If nobody is assigned, move to next one
            if len(assigned_handler) == 0:
                continue

            # Put goalie in the middle if possible
            if len(assigned_handler) > 1:
                if goalie in assigned_handler:
                    idx = assigned_handler.index(goalie)

                    if idx != 1:
                        del assigned_handler[idx]
                        assigned_handler.insert(1, goalie)

            center_line = robocup.Line(threat[0], constants.Field.OurGoalSegment.center())

            main.system_state().draw_line(center_line, constants.Colors.Red, "CenterLine")

            # find the angular width that each defender can block.  We then space these out accordingly
            angle_widths = []
            for handler in assigned_handler:
                dist_from_threat = handler.robot.pos.dist_to(threat[0])
                w = min(2.0 * math.atan2(constants.Robot.Radius,
                                         dist_from_threat), 0.15)
                angle_widths.append(w)

            # start on one edge of our available angle coverage and work counter-clockwise,
            # assigning block lines to the bots as we go
            spacing = 0.01 if len(assigned_handler) < 3 else 0.0  # spacing between each bot in radians
            total_angle_coverage = sum(angle_widths) + (len(angle_widths) -
                                                        1) * spacing
            start_vec = center_line.delta().normalized()
            start_vec.rotate(robocup.Point(0, 0), -total_angle_coverage / 2.0)
            for i in range(len(angle_widths)):
                handler = assigned_handler[i]
                w = angle_widths[i]
                start_vec.rotate(robocup.Point(0, 0), w / 2.0)
                handler.block_line = robocup.Line(threat[0],
                                                  threat[0] + start_vec * 10)
                start_vec.rotate(robocup.Point(0, 0), w / 2.0 + spacing)


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
                    r.robot_change_cost = DefenseRewrite.DEFENSE_ROBOT_CHANGE_COST

        return reqs