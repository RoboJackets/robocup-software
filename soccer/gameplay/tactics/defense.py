import composite_behavior
import behavior
import constants
import robocup
import evaluation.window_evaluator
import main
from enum import Enum
import math
import tactics.positions.goalie
import tactics.positions.defender


class Defense(composite_behavior.CompositeBehavior):


    # defender_priorities should have a length of two and contains the priorities for the two defender
    def __init__(self):
        super().__init__(continuous=True, defender_priorities=[20, 19])


        # we could make the Defense tactic have more or less defenders, but right now we only support two
        if len(defender_priorities) != 2:
            raise RuntimeError("defender_priorities should have a length of two")


        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda:True,
            "immediately")

        goalie = tactics.positions.goalie.Goalie()
        goalie.shell_id = main.root_play().goalie_id
        self.add_subbehavior(goalie, "goalie", required=True)


        # add defenders at the specified priority levels
        for num, priority in enumerate(defender_priorities):
            defender = tactics.defender.Defender()
            self.add_subbehavior(defender, 'defender' + str(num), required=False, priority=priority)


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
        if all(bhvr.robot == None for bhvr in [goalie, fullback1, fullback2]):
            return


        # unhandled_threats is an array of (OpponentRobot/Point, threat_score) tuples
        # the threat_score is intended to be a value proportional to the % chance that the scoring shot will come from them,
        # but keep in mind that it's a pretty rough approximation
        # threat_scores are between zero and one
        # The primary threat has a score of 1.0
        unhandled_threats = []


        # secondary threats are those that are somewhat close to our goal and open for a pass
        # if they're farther than this down the field, we don't consider them threats
        threat_max_y = constants.Field.Length/2.0 * 2.0/3.0
        potential_threats = [opp for opp in main.their_robots() if opp.pos.y < threat_max_y]


        # find the primary threat
        # if the ball is not moving OR it's moving towards our goal, it's the primary threat
        # if it's moving, but not towards our goal, the primary threat is the robot on their team most likely to catch it
        if main.ball().vel.mag() > 0.4:
            # the line the ball's moving along
            ball_travel_line = robocup.Line(main.ball().pos, main.ball().pos + main.ball().vel)

            # this is a shot on the goal!
            if evaluation.ball.is_moving_towards_our_goal():
                unhandled_threats.append( (main.ball(), 1.0) )
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
                        dx = (nearest_point - main.ball().pos).mag()
                        dy = (opp.pos - nearest_point).mag()
                        angle = abs(math.atan2(dy, dx))
                        if angle < math.pi / 4.0:
                            potential_receivers.append( (opp, 1.0) )

                # choose the receiver with the smallest angle from the ball travel line
                best_receiver_tuple = min(potential_receivers, key=lambda rcrv_tuple: rcrv_tuple[1])
                if best_receiver_tuple != None:
                    unhandled_threats.append( (best_receiver_tuple[0], 1.0) )

        else:
            # primary threat is the ball or the opponent holding it
            opp_with_ball = evaluation.ball.opponent_with_ball()
            if opp_with_ball != None:
                unhandled_threats.append( (opp_with_ball, 1.0) )
            else:
                unhandled_threats.append( (main.ball().pos, 1.0) )



        # handled_threats is an array of tuples tracking which of our behaviors we have assigned:
        # (threat, [behaviors]), where threat is a tuple, as before: (OpponentRobot/pos, threat_score)
        # It is sorted in order of importance: primary threat first, secondary threats later
        handled_threats = []

        # available behaviors we have to assign to threats
        # only look at ones that have robots
        # as we handle threats, we remove the handlers from this list
        unused_threat_handlers = filter([goalie, fullback1, fullback2], key=lambda bhvr: bhvr.robot != None)


        # assign goalie to handle the primary threat
        primary_threat = max(unhandled_threats, key=lambda t: t[1])


        # assign a threat handler to the primary threat
        handled_threats.append( (primary_threat, [unused_threat_handlers[0]] ))
        del unused_threat_handlers[0]


        # if an opponent has the ball or is potentially about to receive the ball,
        # we look at potential receivers of it as threats
        if isinstance(primary_threat, robocup.OpponentRobot):
            for opp in filter(potential_threats, key=visible):
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
                windows, best = win_eval.run_pt_to_seg(main.ball().pos, receive_seg)

                # this is our estimate of the likelihood of the pass succeeding
                # value can range from zero to one
                # we square the ratio of best to total to make it weigh more - we could raise it to higher power if we wanted
                if best != None:
                    pass_chance = 0.8 * (best.segment.length() / receive_seg.length())**2
                else:
                    # give it a small chance because the obstacles in the way could move soon and we don't want to consider it a zero threat
                    pass_chance = 0.2


                # Now we evaluate this opponent's shot on the goal
                # exclude robots that have already been assigned to handle other threats
                shot_chance = evaluation.shot.eval_chance(
                    pos=opp.pos,
                    target=constants.Field.OurGoalSegment,
                    windowing_excludes=map(unused_threat_handlers, key=robot),
                    debug=self.debug)

                if shot_chance == 0:
                    # gve it a small chance because the shot could clear up a bit later and we don't want to consider it a zero threat
                    shot_chance = 0.2

                # record the threat
                threat_score = pass_chance * shot_chance
                unhandled_threats.append( (opp, threat_score) )

        else:
            # the ball isn't possessed by an opponent, so we just look at opponents with shots on the goal
            for opp in potential_threats:
                chance_of_getting_ball = 0.2 # this is a bullshit value

                # if it gets the ball...
                shot_chance = evaluation.shot.eval_chance(
                    pos=opp.pos,
                    target=constants.Field.OurGoalSegment,
                    windowing_excludes=map(unused_threat_handlers, key=robot),
                    debug=self.debug)

                # record the threat
                threat_score = chance_of_getting_ball * shot_chance
                unhandled_threats.append( (opp, threat_score) )



        # prioritize by threat score, highest first
        unhandled_threats.sort(key=lambda threat: threat[1], reverse=True)

        raise NotImplementedError("Assign remaining hadlers to threats")

        raise NotImplementedError("Tell the handlers what to do")

        raise NotImplementedError("Better debug drawing")


    def execute_running(self):
        self.recalculate()
 