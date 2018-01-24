import composite_behavior
import behavior
import constants
import robocup
import evaluation.passing
import main
from enum import Enum
import math
import tactics.positions.submissive_goalie as submissive_goalie
import tactics.positions.submissive_defender as submissive_defender
import role_assignment

# TODO: clear free balls
# TODO: handle the case where the ball is invalid


## The Defense tactic handles goalie and defender placement to defend the goal
# It does lots of window and shot evaluation to figure out which 'threats' are the
# most important to block, then assigns blocking positions to the bots
# The old defense strategy had a goalie and two defenders that didn't coordinate with eachother
# and tended to overlap and not get an optimal positioning - this tactic handles the coordination.
class Defense(composite_behavior.CompositeBehavior):

    DEFENSE_ROBOT_CHANGE_COST = 0.29

    class State(Enum):
        ## gets between a particular opponent and the goal.  stays closer to the goal
        defending = 1
        clearing = 2  #Kick the ball away from the goalzone if it is safe to do so

    # defender_priorities should have a length of two and contains the priorities for the two defender
    def __init__(self, defender_priorities=[20, 19]):
        super().__init__(continuous=True)

        # we could make the Defense tactic have more or less defenders, but right now we only support two
        if len(defender_priorities) != 2:
            raise RuntimeError(
                "defender_priorities should have a length of two")

        self.add_state(Defense.State.defending,
                       behavior.Behavior.State.running)
        self.add_state(Defense.State.clearing, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            Defense.State.defending, lambda: True,
                            "immediately")
        self.add_transition(Defense.State.defending, Defense.State.clearing,
                            lambda: self.should_clear_ball(),
                            "when it is safe to clear the ball")
        self.add_transition(Defense.State.clearing, Defense.State.defending,
                            lambda: not self.should_clear_ball(),
                            "done clearing")

        goalie = submissive_goalie.SubmissiveGoalie()
        goalie.shell_id = main.root_play().goalie_id
        self.add_subbehavior(goalie, "goalie", required=False)

        # add defenders at the specified priority levels
        for num, priority in enumerate(defender_priorities):
            defender = submissive_defender.SubmissiveDefender()
            self.add_subbehavior(defender,
                                 'defender' + str(num + 1),
                                 required=False,
                                 priority=priority)

        self.debug = True

        self.win_eval = robocup.WindowEvaluator(main.system_state())

    ## draws some pretty cool shit on the field if set to True
    # default: True
    @property
    def debug(self):
        return self._debug

    @debug.setter
    def debug(self, value):
        self._debug = value

    def should_clear_ball(self):

        if main.game_state().is_stopped():
            return False

        #Returns true if our robot can reach the ball sooner than the closest opponent
        safe_to_clear = False
        if main.ball().pos.mag() < constants.Field.ArcRadius * 2 and main.ball(
        ).vel.mag() < .75 and not evaluation.ball.is_in_our_goalie_zone():

            defender1 = self.subbehavior_with_name('defender1')
            defender2 = self.subbehavior_with_name('defender2')
            if (defender1.robot != None and defender2.robot != None):
                max_vel = robocup.MotionConstraints.MaxRobotSpeed.value
                max_accel = robocup.MotionConstraints.MaxRobotAccel.value

                for robot in main.system_state().their_robots:
                    their_dist_to_ball = robot.pos.dist_to(main.ball().pos)
                    #if their robot is moving faster than ours, assume it is at its maximum speed, otherwise assume its max speed is the same as ours
                    their_max_vel = max(max_vel, robot.vel.mag())

                #calculate time for the closest opponent to reach ball based on current /vel/pos data * .9 for safety
                their_time_to_ball = (
                    their_dist_to_ball /
                    their_max_vel) * defender1.safety_multiplier

                if their_time_to_ball > evaluation.ball.time_to_ball(
                        defender1.robot) or their_time_to_ball > evaluation.ball.time_to_ball(
                            defender2.robot):
                    safe_to_clear = True

        return safe_to_clear

    def execute_running(self):
        self.recalculate()

        goalie = self.subbehavior_with_name("goalie")
        goalie.shell_id = main.root_play().goalie_id
        if goalie.shell_id is None:
            print("WARNING: No Goalie Selected")
            # raise RuntimeError("Defense tactic requires a goalie id to be set")

            # TODO: move a lot of this code into modules in the evaluation folder

            #main.system_state().draw_circle(robocup.Point(0, 0), constants.Field.ArcRadius * 2,constants.Colors.Red, "Clear Ball")

    def on_enter_clearing(self):
        defender1 = self.subbehavior_with_name('defender1')
        defender1.go_clear = True

    def on_exit_clearing(self):
        defender1 = self.subbehavior_with_name('defender1')
        defender1.go_clear = False

    def recalculate(self):
        goalie = self.subbehavior_with_name('goalie')
        defender1 = self.subbehavior_with_name('defender1')
        defender2 = self.subbehavior_with_name('defender2')
        behaviors = [goalie, defender1, defender2]

        # if we don't have any bots to work with, don't waste time calculating
        if all(bhvr.robot is None for bhvr in behaviors):
            return

        # A threat to our goal - something we'll actively defend against
        class Threat:
            def __init__(self, source=None):
                self.source = source
                self.ball_acquire_chance = 1.0
                self.shot_chance = 1.0
                self.assigned_handlers = []
                self.best_shot_window = None

            # an OpponentRobot or Point
            @property
            def source(self):
                return self._source

            @source.setter
            def source(self, value):
                self._source = value

            # our source can be a Point or an OpponentRobot, this method returns the location of it
            @property
            def pos(self):
                if self.source != None:
                    return self.source if isinstance(
                        self.source, robocup.Point) else self.source.pos

            # a list of our behaviors that will be defending against this threat
            # as of now only Defender and Goalie
            @property
            def assigned_handlers(self):
                return self._assigned_handlers

            @assigned_handlers.setter
            def assigned_handlers(self, value):
                self._assigned_handlers = value

            # our estimate of the chance that this threat will acquire the ball
            # 1.0 if it already has it
            # otherwise, a value from 0 to 1 gauging its likelihood to receive a pass
            @property
            def ball_acquire_chance(self):
                return self._ball_acquire_chance

            @ball_acquire_chance.setter
            def ball_acquire_chance(self, value):
                self._ball_acquire_chance = value

            # our estimate of the chance of this threat making its shot on the goal given that it gets/has the ball
            # NOTE: this is calculated excluding all of our robots on the field as obstacles
            @property
            def shot_chance(self):
                return self._shot_chance

            @shot_chance.setter
            def shot_chance(self, value):
                self._shot_chance = value

            # his best window on our goal
            @property
            def best_shot_window(self):
                return self._best_shot_window

            @best_shot_window.setter
            def best_shot_window(self, value):
                self._best_shot_window = value

            # our assessment of the risk of this threat
            # should be between 0 and 1
            @property
            def score(self):
                return self.ball_acquire_chance * self.shot_chance

            # available behaviors we have to assign to threats
            # only look at ones that have robots
            # as we handle threats, we remove the handlers from this list

        unused_threat_handlers = list(filter(
            lambda bhvr: bhvr.robot is not None, [goalie, defender1, defender2
                                                  ]))

        def set_block_lines_for_threat_handlers(threat):
            if len(threat.assigned_handlers) == 0:
                return

            # make sure goalie is in the middle
            if len(threat.assigned_handlers) > 1:
                if goalie in threat.assigned_handlers:
                    idx = threat.assigned_handlers.index(goalie)
                    if idx != 1:
                        del threat.assigned_handlers[idx]
                        threat.assigned_handlers.insert(1, goalie)

            if threat.best_shot_window is not None:
                center_line = robocup.Line(
                    threat.pos, threat.best_shot_window.segment.center())
            else:
                center_line = robocup.Line(
                    threat.pos, constants.Field.OurGoalSegment.center())

            # find the angular width that each defender can block.  We then space these out accordingly
            angle_widths = []
            for handler in threat.assigned_handlers:
                dist_from_threat = handler.robot.pos.dist_to(threat.pos)
                w = min(2.0 * math.atan2(constants.Robot.Radius,
                                         dist_from_threat), 0.15)
                angle_widths.append(w)

            # start on one edge of our available angle coverage and work counter-clockwise,
            # assigning block lines to the bots as we go
            spacing = 0.01 if len(threat.assigned_handlers) < 3 else 0.0  # spacing between each bot in radians
            total_angle_coverage = sum(angle_widths) + (len(angle_widths) -
                                                        1) * spacing
            start_vec = center_line.delta().normalized()
            start_vec.rotate(robocup.Point(0, 0), -total_angle_coverage / 2.0)
            for i in range(len(angle_widths)):
                handler = threat.assigned_handlers[i]
                w = angle_widths[i]
                start_vec.rotate(robocup.Point(0, 0), w / 2.0)
                handler.block_line = robocup.Line(threat.pos,
                                                  threat.pos + start_vec * 10)
                start_vec.rotate(robocup.Point(0, 0), w / 2.0 + spacing)

        def recalculate_threat_shot(threat_index):
            if not isinstance(threat_index, int):
                raise TypeError("threat_index should be an int")

            # ignore all of our robots
            excluded_robots = list(main.our_robots())

            # behaviors before this threat are counted as obstacles in their TARGET position (where we've
            # assigned them to go, not where they are right now)
            hypothetical_obstacles = []
            for t in threats[0:threat_index]:
                hypothetical_obstacles.extend(map(
                    lambda bhvr: bhvr.move_target, t.assigned_handlers))

            threat = threats[threat_index]
            self.win_eval.excluded_robots.clear()
            for r in excluded_robots:
                self.win_eval.add_excluded_robot(r)
            _, threat.best_shot_window = self.win_eval.eval_pt_to_our_goal(
                threat.pos)
            if threat.best_shot_window is not None:
                threat.shot_chance = threat.best_shot_window.shot_success
            else:
                threat.shot_chance = 0.0

        threats = []

        # TODO figure out which threats are moving down the field and adjust on that
        # Also see ER-Force's ETDP from 2017 for more information
        threat_max_y = constants.Field.Length
        potential_threats = [opp
                             for opp in main.their_robots()
                             if opp.pos.y < threat_max_y]

        # find the primary threat
        # if the ball is not moving OR it's moving towards our goal, it's the primary threat
        # if it's moving, but not towards our goal, the primary threat is the robot on their team most likely to catch it
        if main.ball().vel.mag() > 0.4:
            # the line the ball's moving along
            ball_travel_line = robocup.Line(main.ball().pos,
                                            main.ball().pos + main.ball().vel)

            # this is a shot on the goal!
            if evaluation.ball.is_moving_towards_our_goal():
                ball_threat = Threat(main.ball().pos)
                ball_threat.ball_acquire_chance = 1.0
                ball_threat.shot_chance = 1.0
                threats.append(ball_threat)
            else:
                # Check for a bot that's about to capture this ball and potentially shoot on the goal
                # potential_receivers is an array of (OpponentRobot, angle) tuples, where the angle
                # is the angle between the ball_travel_line and the line from the ball to the opponent
                # bot - this is our metric for receiver likeliness.
                potential_receivers = []
                for opp in potential_threats:
                    # see if the bot is in the direction the ball is moving
                    if (opp.pos - ball_travel_line.get_pt(0)).dot(
                            ball_travel_line.delta()) > 0:
                        # calculate the angle and add it to the list if it's within reason
                        nearest_pt = ball_travel_line.nearest_point(opp.pos)
                        dx = (nearest_pt - main.ball().pos).mag()
                        dy = (opp.pos - nearest_pt).mag()
                        angle = abs(math.atan2(dy, dx))
                        if angle < math.pi / 4.0:
                            potential_receivers.append((opp, 1.0))

                # choose the receiver with the smallest angle from the ball travel line
                if len(potential_receivers) > 0:
                    best_receiver_tuple = min(
                        potential_receivers,
                        key=lambda rcrv_tuple: rcrv_tuple[1])
                    if best_receiver_tuple != None:
                        receiver_threat = Threat(best_receiver_tuple[0])
                        receiver_threat.ball_acquire_chance = 0.9  # note: this value is arbitrary
                        receiver_threat.shot_chance = 0.9  # FIXME: calculate this
                        threats.append(receiver_threat)
                else:
                    ball_threat = Threat(main.ball().pos)
                    ball_threat.ball_acquire_chance = 1.0
                    ball_threat.shot_chance = 0.9
                    threats.append(ball_threat)

        else:

            if not constants.Field.OurGoalZoneShape.contains_point(main.ball(
            ).pos):

                # primary threat is the ball or the opponent holding it
                opp_with_ball = evaluation.ball.opponent_with_ball()

                threat = Threat(opp_with_ball if opp_with_ball is not None else
                                main.ball().pos)
                threat.ball_acquire_chance = 1.0
                threat.shot_chance = 1.0  # FIXME: calculate, don't use 1.0
                threats.append(threat)

        # if an opponent has the ball or is potentially about to receive the ball,
        # we look at potential receivers of it as threats
        if len(threats) > 0 and isinstance(threats[0].source,
                                           robocup.OpponentRobot):
            for opp in filter(lambda t: t.visible, potential_threats):
                pass_chance = evaluation.passing.eval_pass(
                    main.ball().pos,
                    opp.pos,
                    excluded_robots=[opp])
                # give it a small chance because the obstacles in the way could move soon and we don't want to consider it a zero threatos, )
                if pass_chance < 0.001: pass_chance = 0.4

                # record the threat
                threat = Threat(opp)
                threat.ball_acquire_chance = pass_chance
                threats.append(threat)

                # Now we evaluate this opponent's shot on the goal
                # exclude robots that have already been assigned to handle other threats
                self.win_eval.excluded_robots.clear()
                for r in map(lambda bhvr: bhvr.robot, unused_threat_handlers):
                    self.win_eval.add_excluded_robot(r)
                _, threat.best_shot_window = self.win_eval.eval_pt_to_our_goal(
                    opp.pos)
                if threat.best_shot_window is not None:
                    threat.shot_chance = threat.best_shot_window.shot_success
                else:
                    threat.shot_chance = 0.0

                if threat.shot_chance == 0:
                    # gve it a small chance because the shot could clear up a bit later and we don't want to consider it a zero threat
                    threat.shot_chance = 0.2

        else:
            # the ball isn't possessed by an opponent, so we just look at opponents with shots on the goal
            for opp in potential_threats:
                # record the threat
                lurker = Threat(opp)
                lurker.ball_acquire_chance = 0.5  # note: this is a bullshit value
                threats.append(lurker)
                recalculate_threat_shot(len(threats) - 1)

        # only consider the top three threats
        threats.sort(key=lambda threat: threat.score, reverse=True)
        threats = threats[0:3]

        # print("sorted threats:")
        # for idx, t in enumerate(threats):
        #     print("t[" + str(idx) + "]: " + str(t.source) + "shot: " + str(t.shot_chance) + "; pass:" + str(t.ball_acquire_chance) + "; score:" + str(t.score))

        # print("sorted threat scores: " + str(list(map(lambda t: str(t.score) + " " + str(t.source), threats))))

        # print("Unused handlers: " + str(unused_threat_handlers))
        # print("---------------------")

        # If we have nothing to block, bail
        if not threats:
            return


        # only deal with top two threats
        threats_to_block = threats[0:2]

        # print('threats to block: ' + str(list(map(lambda t: t.source, threats_to_block))))

        # If we clearing the ball, assign the clearer to the most important
        # threat (the ball). This prevents assigning the non-clearing robot
        # to mark the ball and causing crowding.
        defender1 = self.subbehavior_with_name('defender1')
        if (defender1.state ==
                submissive_defender.SubmissiveDefender.State.clearing):
            if defender1 in unused_threat_handlers:
                if (threats_to_block[0].pos.dist_to(main.ball().pos) <
                        constants.Robot.Radius * 2):
                    defender_idx = unused_threat_handlers.index(defender1)
                    threats_to_block[0].assigned_handlers.append(
                        unused_threat_handlers[defender_idx])
                    del unused_threat_handlers[defender_idx]

        threat_idx = 0
        while len(unused_threat_handlers) > 0:
            threats_to_block[threat_idx].assigned_handlers.append(
                unused_threat_handlers[0])
            del unused_threat_handlers[0]

            threat_idx = (threat_idx + 1) % len(threats_to_block)

        for t_idx, t in enumerate(threats_to_block):
            recalculate_threat_shot(t_idx)
            set_block_lines_for_threat_handlers(t)


        # tell the bots where to move / what to block and draw some debug stuff
        for idx, threat in enumerate(threats):

            # recalculate, including all current bots
            # FIXME: do we want this?
            # recalculate_threat_shot(idx)

            # the line they'll be shooting down/on
            if threat.best_shot_window is not None:
                shot_line = robocup.Segment(
                    threat.pos, threat.best_shot_window.segment.center())
            else:
                shot_line = robocup.Segment(threat.pos, robocup.Point(0, 0))

            # debug output
            if self.debug:
                for handler in threat.assigned_handlers:
                    # handler.robot.add_text("Marking: " + str(threat.source), constants.Colors.White, "Defense")
                    main.system_state().draw_circle(handler.move_target, 0.02,
                                                    constants.Colors.Blue,
                                                    "Defense")

                # draw some debug stuff
                if threat.best_shot_window is not None:
                    # draw shot triangle
                    pts = [threat.pos,
                           threat.best_shot_window.segment.get_pt(0),
                           threat.best_shot_window.segment.get_pt(1)]
                    shot_color = (255, 0, 0, 150)  # translucent red
                    main.system_state().draw_polygon(pts, shot_color,
                                                     "Defense")
                    main.system_state().draw_segment(
                        threat.best_shot_window.segment, constants.Colors.Red,
                        "Defense")

                    self.win_eval.excluded_robots.clear()
                    _, best_window = self.win_eval.eval_pt_to_our_goal(
                        threat.pos)
                    if best_window is not None:
                        chance = best_window.shot_success
                    else:
                        chance = 0.0

                    main.system_state().draw_text(
                        "Shot: " + str(int(threat.shot_chance * 100.0)) +
                        "% / " + str(int(chance * 100)) + "%",
                        shot_line.center(), constants.Colors.White, "Defense")

                # draw pass lines
                if idx > 0:
                    pass_line = robocup.Segment(main.ball().pos, threat.pos)
                    main.system_state().draw_line(
                        pass_line, constants.Colors.Red, "Defense")
                    main.system_state().draw_text(
                        "Pass: " + str(int(threat.ball_acquire_chance * 100.0))
                        + "%", pass_line.center(), constants.Colors.White,
                        "Defense")

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
