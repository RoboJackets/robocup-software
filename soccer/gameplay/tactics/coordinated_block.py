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


##
#
# A tactic that coordinates a submissive goalie with two
# defensive bots to block threats to the goal
#
# Note that this tactic handles the goalie, so the handles goalie
# function in standard play must be overridden to reflect that
#
class CoordinatedBlock(composite_behavior.CompositeBehavior):

    DEFENSE_ROBOT_CHANGE_COST = 0.05

    class State(Enum):
        # Gets in the way of the opponent robots
        defending = 1

    def __init__(self, defender_priorities=[20, 19]):
        super().__init__(continuous=True)

        if len(defender_priorities) != 2:
            raise RuntimeError("defender_priorities should have a length of 2")

        self.add_state(CoordinatedBlock.State.defending,
                       behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            CoordinatedBlock.State.defending, lambda: True,
                            "immediately")

        goalie = submissive_goalie.SubmissiveGoalie()
        goalie.shell_id = main.root_play().goalie_id
        self.add_subbehavior(goalie, "goalie", required=False)

        # Add the defenders
        for num, priority in enumerate(defender_priorities):
            defender = submissive_defender.SubmissiveDefender()
            self.add_subbehavior(
                defender,
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

    def execute_running(self):
        goalie = self.subbehavior_with_name("goalie")
        goalie.shell_id = main.root_play().goalie_id

        if goalie.shell_id is None:
            print("WARNING: No Goalie Selected")

        self.find_and_set_defender_location()

    def find_and_set_defender_location(self):
        goalie = self.subbehavior_with_name('goalie')
        defender1 = self.subbehavior_with_name('defender1')
        defender2 = self.subbehavior_with_name('defender2')
        unused_threat_handlers = list(
            filter(lambda bhvr: bhvr.robot is not None,
                   [goalie, defender1, defender2]))

        threats = self.get_threat_list(unused_threat_handlers)

        # If no threats, kick out
        if not threats:
            return

        # Get top 2 threats based on score
        threats.sort(key=lambda threat: threat[1], reverse=True)
        threats_to_block = threats[0:2]
        assigned_handlers = [[], []]

        self.assign_handlers_to_threats(
            assigned_handlers, unused_threat_handlers, threats_to_block)

        self.set_defender_block_lines(threats_to_block, assigned_handlers)

    ## Gets list of threats
    #  @return tuple of threat positions and score (unordered)
    def get_threat_list(self, unused_threat_handlers):
        # List of (position, score, Robot/None)
        threats = []
        potential_threats = main.their_robots()

        # find the primary threat
        # if the ball is not moving OR it's moving towards our goal, it's the primary threat
        # if it's moving, but not towards our goal, the primary threat is the robot on their team most likely to catch it
        if (main.ball().vel.mag() > 0.4):
            if evaluation.ball.is_moving_towards_our_goal():
                # Add tuple of pos and score
                threats.append((main.ball().pos, 1, None))
            else:
                # Get all potential receivers
                potential_receivers = []
                for opp in potential_threats:
                    if self.estimate_potential_recievers_score(opp) == 1:
                        potential_receivers.append((opp.pos, 1, opp))

                if len(potential_receivers) > 0:
                    # Add best receiver to threats
                    # TODO Calc shot chance
                    best_tuple = min(potential_receivers,
                                     key=lambda rcrv_tuple: rcrv_tuple[1])
                    threats.append((best_tuple[0], .81, best_tuple[2]))
                else:
                    # Just deal with ball if no recievers
                    threats.append((main.ball().pos, .9, None))
        else:
            # Assume opp is dribbling ball
            if not constants.Field.OurGoalZoneShape.contains_point(
                    main.ball().pos):
                # TODO: Calc shot chance
                threats.append((main.ball().pos, 1, None))

        # if there are threats, check pass and shot chances
        # If the first item is not a ball, it is most likely a pass
        if len(threats) > 0 and threats[0][0] != main.ball().pos:
            for opp in potential_threats:

                # Exclude robots that have been assigned already
                excluded_bots = []
                for r in map(lambda bhvr: bhvr.robot, unused_threat_handlers):
                    excluded_bots.append(r)

                threats.append((opp.pos, self.estimate_risk_score(
                    opp, excluded_bots), opp))
        else:
            for opp in potential_threats:

                # Exclude all robots
                self.kick_eval.excluded_robots.clear()
                self.kick_eval.add_excluded_robot(opp)
                for r in main.our_robots():
                    self.kick_eval.add_excluded_robot(r)

                point, shotChance = self.kick_eval.eval_pt_to_our_goal(opp.pos)

                # Note: 0.5 is a bullshit value
                threats.append((opp.pos, 0.5 * shotChance, opp))

        # Prevent threats from being below our goal line (causes incorrect pos)
        def _adjust_pt(threat):
            pt = threat[0]
            pt.y = max(pt.y, 0.1)
            return (pt,) + threat[1:]

        threats = list(map(_adjust_pt, threats))

        return threats

    ## Estimate risk score based on old defense.py play
    #  @param bot Robot to estimate score at
    #  @param exluded_Bots Robots to exclude from the defense when calculating shot
    #  @return The risk score at that point (Shot chance * pass chance)
    def estimate_risk_score(self, bot, excluded_bots=[]):
        excluded_bots.append(bot)

        passChance = evaluation.passing.eval_pass(
            main.ball().pos, bot.pos, excluded_robots=excluded_bots)

        self.kick_eval.excluded_robots.clear()
        for r in excluded_bots:
            self.kick_eval.add_excluded_robot(r)

        point, shotChance = self.kick_eval.eval_pt_to_our_goal(bot.pos)

        return passChance * shotChance

    ## Estimate potential reciever score based on old defense.py play
    #  @param bot Robot to estimate score at
    #  @return The potential receiver score at that point
    def estimate_potential_recievers_score(self, bot):
        ball_travel_line = robocup.Line(main.ball().pos,
                                        main.ball().pos + main.ball().vel)

        dot_product = (bot.pos - main.ball().pos).dot(ball_travel_line.delta())
        nearest_pt = ball_travel_line.nearest_point(bot.pos)
        dx = (nearest_pt - main.ball().pos).mag()
        dy = (bot.pos - nearest_pt).mag()
        angle = abs(math.atan2(dy, dx))

        # Only returns 1 if the opp is moving in the opposite direction as the ball
        # and the angle between the ball ray starting at its current position and the opp position
        # is less than pi/4
        if (angle < math.pi / 4 and dot_product > 0):
            return 1
        else:
            return 0

    ## Assigns the defenders to threats
    #  @param assigned_handlers List of list, [ A ... ] where A represents a list of defenders assigned to threat A
    #  @param unused_threat_handlers List of defenders that are unused currently
    #  @param threats_to_block List of threats that we have to deal with, tuple with position and threat score
    def assign_handlers_to_threats(self, assigned_handlers,
                                   unused_threat_handlers, threats_to_block):
        threat_idx = 0
        while len(unused_threat_handlers) > 0:
            assigned_handlers[threat_idx].append(unused_threat_handlers[0])
            del unused_threat_handlers[0]

            threat_idx = (threat_idx + 1) % len(threats_to_block)

    ## Assigns the locations for each robot to block given a threat and list of robots to block each threat
    #  @param threats_to_block List of threats that we have to deal with, tuple with position and threat score
    #  @param assigned_handlers List of list, [ A ... ] where A represents a list of defenders assigned to threat A
    def set_defender_block_lines(self, threats_to_block, assigned_handlers):
        goalie = self.subbehavior_with_name('goalie')
        defender1 = self.subbehavior_with_name('defender1')
        defender2 = self.subbehavior_with_name('defender2')

        # Check keep defenders from occupying the same spot
        # it will break if you change this
        handlers = [goalie, defender1, defender2]

        # For each threat
        for threat_idx in range(len(threats_to_block)):
            # Get the threat pos and score
            threat = threats_to_block[threat_idx]
            # Grab the list of handlers assigned to this threat
            assigned_handler = assigned_handlers[threat_idx]

            # Exclude any robots we are about to assign to find the threats best shot
            self.kick_eval.excluded_robots.clear()
            for handler in handlers:
                self.kick_eval.add_excluded_robot(handler.robot)

            # Add opp robot into exclude list, assuming it is not a ball
            if (threat[2] is not None):
                self.kick_eval.add_excluded_robot(threat[2])

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

            # Get best shot from that threat postiion
            point, shot_chance = self.kick_eval.eval_pt_to_our_goal(threat[0])
            shot_line = robocup.Line(threat[0], point)

            # find the angular width that each defender can block.  We then space these out accordingly
            angle_widths = []
            for handler in assigned_handler:
                dist_from_threat = handler.robot.pos.dist_to(threat[0])
                w = min(2.0 * math.atan2(constants.Robot.Radius,
                                         dist_from_threat), 0.15)
                angle_widths.append(w)

            # start on one edge of our available angle coverage and work counter-clockwise,
            # assigning block lines to the bots as we go
            spacing = 0.01 if len(
                assigned_handler
            ) < 3 else 0.0  # spacing between each bot in radians
            total_angle_coverage = sum(angle_widths) + (len(angle_widths) - 1
                                                        ) * spacing
            start_vec = shot_line.delta().normalized()
            start_vec.rotate(robocup.Point(0, 0), -total_angle_coverage / 2.0)
            for i in range(len(angle_widths)):
                handler = assigned_handler[i]
                w = angle_widths[i]
                start_vec.rotate(robocup.Point(0, 0), w / 2.0)
                handler.block_line = robocup.Line(threat[0],
                                                  threat[0] + start_vec * 10)
                start_vec.rotate(robocup.Point(0, 0), w / 2.0 + spacing)

            # Draw all the debug stuff
            if self.debug:
                main.debug_drawer().draw_line(shot_line, constants.Colors.Red,
                                              "Defense-Shot Line")
                main.debug_drawer().draw_text(
                    "Shot: " + str(int(shot_chance * 100.0)), threat[0],
                    constants.Colors.White, "Defense-Shot Percent")

                # Other threats besides ball
                if threat_idx > 0:
                    pass_line = robocup.Segment(main.ball().pos, threat[0])
                    main.debug_drawer().draw_line(
                        pass_line, constants.Colors.Red, "Defense-Pass Line")

            # keep defenders from occupying the same spot
            # only matters if there are 2 defenders (and the goalie)
            if len(handlers) == 3:
                handler1 = handlers[1]
                handler2 = handlers[2]

                #vector between the 2 points
                overlap = handler2.move_target - handler1.move_target

                #if the robots overlap
                if overlap.mag() < (2 * constants.Robot.Radius) + .005:
                    #move the robots away from each other
                    overlap = overlap - (overlap.normalized() * 1.8 *
                                         constants.Robot.Radius)

                    handler1._move_target += overlap
                    handler2._move_target -= overlap

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
                    r.robot_change_cost = CoordinatedBlock.DEFENSE_ROBOT_CHANGE_COST

        return reqs
