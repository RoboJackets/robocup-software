import composite_behavior
import behavior
import constants
import robocup
import evaluation.window_evaluator
import evaluation.shot
import main
from enum import Enum
import math
import tactics.positions.submissive_goalie
import tactics.positions.submissive_defender


# TODO: clear free balls
# TODO: handle the case where the ball is invalid


# The Defense tactic handles goalie and defender placement to defend the goal
# It does lots of window and shot evaluation to figure out which 'threats' are the
# most important to block, then assigns blocking positions to the bots
# The old defense strategy had a goalie and two defenders that didn't coordinate with eachother
# and tended to overlap and not get an optimal positioning - this tactic handles the coordination
class Defense(composite_behavior.CompositeBehavior):

    # defender_priorities should have a length of two and contains the priorities for the two defender
    def __init__(self, defender_priorities=[20, 19]):
        super().__init__(continuous=True)


        # we could make the Defense tactic have more or less defenders, but right now we only support two
        if len(defender_priorities) != 2:
            raise RuntimeError("defender_priorities should have a length of two")


        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda:True,
            "immediately")

        goalie = tactics.positions.submissive_goalie.SubmissiveGoalie()
        goalie.shell_id = main.root_play().goalie_id
        self.add_subbehavior(goalie, "goalie", required=True)


        # add defenders at the specified priority levels
        for num, priority in enumerate(defender_priorities):
            defender = tactics.positions.submissive_defender.SubmissiveDefender()
            self.add_subbehavior(defender, 'defender' + str(num+1), required=False, priority=priority)


        self.debug = True



    # draws some pretty cool shit on the field if set to True
    # default: True
    @property
    def debug(self):
        return self._debug
    @debug.setter
    def debug(self, value):
        self._debug = value
    


    def execute_running(self):
        self.recalculate()

        goalie = self.subbehavior_with_name("goalie")
        goalie.shell_id = main.root_play().goalie_id
        if goalie.shell_id == None:
            raise RuntimeError("Defense tactic requires a goalie id to be set")


    # TODO: move a lot of this code into modules in the evaluation folder
    def recalculate(self):
        goalie = self.subbehavior_with_name('goalie')
        defender1 = self.subbehavior_with_name('defender1')
        defender2 = self.subbehavior_with_name('defender2')

        # if we don't have any bots to work with, don't waste time calculating
        if all(bhvr.robot == None for bhvr in [goalie, defender1, defender2]):
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
                    return self.source if isinstance(self.source, robocup.Point) else self.source.pos
            

            # a list of our behaviors that will be defending against this threat
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
        unused_threat_handlers = list(filter(lambda bhvr: bhvr.robot != None, [goalie, defender1, defender2]))



        def set_block_lines_for_threat_handlers(threat):
            if len(threat.assigned_handlers) == 0:
                return

            print("Setting block lines\n------------------")

            if threat.best_shot_window != None:
                center_line = robocup.Line(threat.pos, threat.best_shot_window.segment.center())
            else:
                center_line = robocup.Line(threat.pos, constants.Field.OurGoalSegment.center())

            # find the angular width that each defender can block.  We then space these out accordingly
            angle_widths = []
            for handler in threat.assigned_handlers:
                dist_from_threat = handler.robot.pos.dist_to(threat.pos)
                w = 2.0 * math.atan2(constants.Robot.Radius, dist_from_threat)
                angle_widths.append(w)


            print("Angle widths: " + str(angle_widths))

            # start on one edge of our available angle coverage and work counter-clockwise,
            # assigning block lines to the bots as we go
            spacing = 0.04  # spacing between each bot in radians
            total_angle_coverage = sum(angle_widths) + (len(angle_widths) - 1)*spacing
            start_vec = center_line.delta().normalized()
            start_vec.rotate(robocup.Point(0,0), (-total_angle_coverage / 2.0) * constants.RadiansToDegrees)
            for i in range(len(angle_widths)):
                w = angle_widths[i]
                start_vec.rotate(robocup.Point(0,0), w/2.0 * constants.RadiansToDegrees)
                handler.block_line = robocup.Line(threat.pos, threat.pos + start_vec * 10)
                start_vec.rotate(robocup.Point(0,0), (w/2.0 + spacing) * constants.RadiansToDegrees)



        def recalculate_threat_shot(threat_index):
            if not isinstance(threat_index, int):
                raise TypeError("threat_index should be an int")

            # ignore all defenders
            excluded_robots = []
            for t in threats:
                excluded_robots.extend(map(lambda bhvr: bhvr.robot, t.assigned_handlers))

            # behaviors before this threat are counted as obstacles in their TARGET position (where we've
            # assigned them to go, not where they are right now)
            hypothetical_obstacles = []
            for t in threats[0:threat_index]:
                hypothetical_obstacles.extend(map(lambda bhvr: bhvr.move_target, t.assigned_handlers))

            threat = threats[threat_index]
            threat.shot_chance, threat.best_shot_window = evaluation.shot.eval_shot(
                pos=threat.pos,
                target=constants.Field.OurGoalSegment,
                windowing_excludes=excluded_robots,
                hypothetical_robot_locations=hypothetical_obstacles,
                debug=False)


        threats = []


        # secondary threats are those that are somewhat close to our goal and open for a pass
        # if they're farther than this down the field, we don't consider them threats
        threat_max_y = constants.Field.Length / 2.0
        potential_threats = [opp for opp in main.their_robots() if opp.pos.y < threat_max_y]


        # find the primary threat
        # if the ball is not moving OR it's moving towards our goal, it's the primary threat
        # if it's moving, but not towards our goal, the primary threat is the robot on their team most likely to catch it
        if main.ball().vel.mag() > 0.4:
            # the line the ball's moving along
            ball_travel_line = robocup.Line(main.ball().pos, main.ball().pos + main.ball().vel)

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
                    if (opp.pos - ball_travel_line.get_pt(0)).dot(ball_travel_line.delta()) > 0:
                        # calculate the angle and add it to the list if it's within reason
                        nearest_pt = ball_travel_line.nearest_point(opp.pos)
                        dx = (nearest_pt - main.ball().pos).mag()
                        dy = (opp.pos - nearest_pt).mag()
                        angle = abs(math.atan2(dy, dx))
                        if angle < math.pi / 4.0:
                            potential_receivers.append( (opp, 1.0) )

                # choose the receiver with the smallest angle from the ball travel line
                if len(potential_receivers) > 0:
                    best_receiver_tuple = min(potential_receivers, key=lambda rcrv_tuple: rcrv_tuple[1])
                    if best_receiver_tuple != None:
                        receiver_threat = Threat(best_receiver_tuple[0])
                        receiver_threat.ball_acquire_chance = 0.9   # note: this value is arbitrary
                        receiver_threat.shot_chance = 0.9   # FIXME: calculate this
                        threats.append(receiver_threat)

        else:
            # primary threat is the ball or the opponent holding it
            opp_with_ball = evaluation.ball.opponent_with_ball()

            threat = Threat(opp_with_ball if opp_with_ball != None else main.ball().pos)
            threat.ball_acquire_chance = 1.0
            threat.shot_chance = 1.0    # FIXME: calculate, don't use 1.0
            threats.append(threat)

 

        # if an opponent has the ball or is potentially about to receive the ball,
        # we look at potential receivers of it as threats
        if isinstance(threats[0].source, robocup.OpponentRobot):
            for opp in filter(lambda t: t.visible, potential_threats):
                # we make a pass triangle with the far corner at the ball and the opposing side touching the receiver's mouth
                # the side along the receiver's mouth is the 'receive_seg'
                # we then use the window evaluator on this scenario to see if the pass is open
                pass_angle = math.pi / 8.0
                pass_dist = opp.pos.dist_to(main.ball().pos)
                pass_dir = opp.pos - main.ball().pos
                pass_perp = pass_dir.perp_ccw()
                receive_point = opp.pos - pass_dir * constants.Robot.Radius # the mouth of the receiver
                receive_seg_half_len = math.tan(pass_angle) * pass_dist
                receive_seg = robocup.Segment(receive_point + pass_perp*receive_seg_half_len,
                    receive_point + pass_perp*-receive_seg_half_len)

                win_eval = evaluation.window_evaluator.WindowEvaluator()
                win_eval.excluded_robots = [opp]
                windows, best = win_eval.eval_pt_to_seg(main.ball().pos, receive_seg)

                # this is our estimate of the likelihood of the pass succeeding
                # value can range from zero to one
                # we square the ratio of best to total to make it weigh more - we could raise it to higher power if we wanted
                if best != None:
                    pass_chance = 0.8 * (best.segment.length() / receive_seg.length())**2
                else:
                    # give it a small chance because the obstacles in the way could move soon and we don't want to consider it a zero threat
                    pass_chance = 0.4


                # record the threat
                threat = Threat(opp)
                threat.ball_acquire_chance = pass_chance
                threats.append(threat)

                # Now we evaluate this opponent's shot on the goal
                # exclude robots that have already been assigned to handle other threats
                threat.shot_chance, threat.best_shot_window = evaluation.shot.eval_shot(
                    pos=opp.pos,
                    target=constants.Field.OurGoalSegment,
                    windowing_excludes=map(lambda bhvr: bhvr.robot, unused_threat_handlers),
                    debug=False)

                if threat.shot_chance == 0:
                    # gve it a small chance because the shot could clear up a bit later and we don't want to consider it a zero threat
                    threat.shot_chance = 0.2


        else:
            # the ball isn't possessed by an opponent, so we just look at opponents with shots on the goal
            for opp in potential_threats:
                # record the threat
                lurker = Threat(opp)
                lurker.ball_acquire_chance = 0.5 # note: this is a bullshit value
                threats.append(lurker)
                recalculate_threat_shot(len(threats) - 1)


        # only consider the top three threats
        threats.sort(key=lambda threat: threat.score, reverse=True)
        threats = threats[0:3]


        # assign all of our defenders to do something
        while len(unused_threat_handlers) > 0:
            # prioritize by threat score, highest first
            top_threat = max(threats, key=lambda threat: threat.score)

            # assign the next handler to this threat
            handler = unused_threat_handlers[0]
            top_threat.assigned_handlers.append(handler)
            del unused_threat_handlers[0]

            # reassign the block line for each handler of this threat
            set_block_lines_for_threat_handlers(top_threat)

            # recalculate the shot now that we have
            recalculate_threat_shot(0)


        # tell the bots where to move / what to block and draw some debug stuff
        for idx, threat in enumerate(threats):

            # recalculate, including all current bots
            # FIXME: do we want this?
            # recalculate_threat_shot(idx)

            # the line they'll be shooting down/on
            if threat.best_shot_window != None:
                shot_line = robocup.Segment(threat.pos, threat.best_shot_window.segment.center())
            else:
                shot_line = robocup.Segment(threat.pos, robocup.Point(0, 0))


            # debug output
            for handler in threat.assigned_handlers:
                handler.robot.add_text("Marking: " + str(threat.source), constants.Colors.White, "Defense")
                # FIXME: choose block_objects collaboratively (so there's no overlap)
                handler.block_line = shot_line
                main.system_state().draw_line(handler.block_line, constants.Colors.Blue, "Defense")


            # draw some debug stuff
            if threat.best_shot_window != None:
                for i in range(2):
                    edge = robocup.Segment(threat.pos, threat.best_shot_window.segment.get_pt(i))
                    main.system_state().draw_line(edge, constants.Colors.Red, "Defense")
                main.system_state().draw_line(threat.best_shot_window.segment, constants.Colors.Red, "Defense")

                chance, best_window = evaluation.shot.eval_shot(threat.pos, constants.Field.OurGoalSegment)

                main.system_state().draw_text("Shot: " + str(int(threat.shot_chance * 100.0)) + "% / " + str(int(chance*100)) + "%", shot_line.center(), constants.Colors.White, "Defense")
            
            # draw pass lines
            if idx > 0:
                pass_line = robocup.Segment(main.ball().pos, threat.pos)
                main.system_state().draw_line(pass_line, constants.Colors.Red, "Defense")
                main.system_state().draw_text("Pass: " + str(int(threat.ball_acquire_chance * 100.0)) + "%", pass_line.center(), constants.Colors.White, "Defense")


        self.threats = threats



    # def __str__(self):
    #     desc = super().__str__()
    #     for t in threats:
    #         desc += 

