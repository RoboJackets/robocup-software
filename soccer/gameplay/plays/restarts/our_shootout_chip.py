import standard_play
import behavior
import skills.move
import skills.pivot_kick
import skills.dribble
import constants
import robocup
import math
import main
import plays.restarts.our_free_kick as our_free_kick
import tactics.coordinated_pass
import evaluation.passing_positioning
from enum import Enum
import play
import time

class OurShootoutChip(play.Play):

    #currently not sure if these variables are necessary will update soon
    BumpKickPower = 0.01
    FullKickPower = 1
    MaxShootingAngle = 80

    class State(Enum):
        # set up before the shoot out behind the ball
        starting = 0
        # move the ball up 1 meter
        dribbling = 1
        # chip into goal
        chipping = 2
        # intead shoot into goal
        shooting = 3
        # if still viable recapture the ball
        capture = 4

    def __init__(self):
        super().__init__(continuous=True)

        #initalize states
        for s in OurShootoutChip.State :
            self.add_state(s, behavior.Behavior.State.running)

        # add transition to lead into the first state (starting/setup state)
        self.add_transition(behavior.Behavior.State.start,
                            OurShootoutChip.State.starting, lambda: True,
                            'immediately')

        # one the ball is able to be hit begin dribbling a meter up
        self.add_transition(OurShootoutChip.State.starting,
                            OurShootoutChip.State.dribbling,
                            lambda: main.game_state().is_ready_state(),
                            'begin')

        # if the goalie gets too close chip over him
        self.add_transition(OurShootoutChip.State.dribbling,
                            OurShootoutChip.State.chipping, lambda: self.is_goalie_close(),
                            'goalie in range')

        # if the goalie is not too close and we reach a "must shoot time" (since we only have 10s)
        # just kick the ball into goal
        self.add_transition(OurShootoutChip.State.dribbling,
                            OurShootoutChip.State.shooting,
                            lambda: not self.is_goalie_close() and self.must_shoot_time(),
                            'must shoot')

        # if the ball fails to go in after a chip try to capture
        self.add_transition(OurShootoutChip.State.chipping, 
                            OurShootoutChip.State.capture,
                            lambda: self.has_subbehavior_with_name('chipping') and 
                            self.subbehavior_with_name('chipping').state == behavior.Behavior.State.completed,
                            'recapture')

        # after you capture shoot the ball with a kick
        self.add_transition(OurShootoutChip.State.capture,
                            OurShootoutChip.State.shooting,
                            lambda:self.subbehavior_with_name('capture').state == behavior.Behavior.State.completed,
                            'reshoot')

        self.total_time = 10.0
        self.shoot_time = 6.0
        self.start_time = time.time()
        self.chip_distance = 1.5
        self.goalie_range = 2

    @classmethod
    def handles_goalie(cls):
        return True

    @classmethod
    def is_restart(cls):
        return False

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if gs.is_penalty_shootout() and gs.is_our_penalty() else float("inf")

    def on_enter_starting(self):
        # find the direction from the enemy goal to the ball
        behind_ball = (main.ball().pos - constants.Field.TheirGoalSegment.center())
        # normalize the vector
        behind_ball = behind_ball.normalized()
        # move the robot one robot width behind the ball away from the opponent's goal
        start_point = behind_ball * 2 * constants.Robot.Radius + main.ball().pos
        self.add_subbehavior(skills.move.Move(start_point), 'starting', required = False,
                            priority = 5)

    def on_exit_starting(self):
        self.remove_all_subbehaviors()

    def on_enter_dribbling(self):
        # set the time for the start of the play
        self.start_time = time.time()
        # find the vector of the ball to the goal
        robot_pos = main.ball().pos
        center_goal = constants.Field.TheirGoalSegment.center()
        dribble_point = (center_goal - robot_pos)
        # normalize the vector
        dribble_point = dribble_point.normalized()
        # added that to the current ball position going one meter to the goal
        dribble_point = dribble_point + robot_pos
        # dribble one meter up
        dribble = skills.dribble.Dribble(dribble_point)
        if (not self.has_subbehavior_with_name('dribble')):
            self.add_subbehavior(dribble, 'dribble', required = False, priority = 5)

    def on_exit_dribbling(self):
        self.remove_all_subbehaviors()

    def on_enter_chipping(self):
            # setup chipper
            kicker = skills.line_kick.LineKick()
            kicker.use_chipper = True
            kicker.min_chip_range = our_free_kick.OurFreeKick.MinChipRange
            kicker.max_chip_range = our_free_kick.OurFreeKick.MaxChipRange
            #calculate target
            gap = evaluation.shooting.find_gap(max_shooting_angle= our_free_kick.OurFreeKick.MaxShootingAngle)
            kicker.target = gap
            # chip
            if (not self.has_subbehavior_with_name('chipper')) :
                self.add_subbehavior(kicker, 'chipper', required=False, priority=5)

    def on_exit_chipping(self):
        self.remove_all_subbehaviors()

    def execute_shooting(self):
        kicker = skills.line_kick.LineKick()
        # aim for the goal segmant
        kicker.target = constants.Field.TheirGoalSegment
        # kick
        if (not self.has_subbehavior_with_name('kicker')) :
                self.add_subbehavior(kicker, 'kicker', required=False, priority=5)

    def on_exit_shooting(self):
        self.remove_all_subbehaviors()

    #capture the ball
    def execute_capture (self):
        capture = tactics.capture.Capture()
        if (not self.has_subbehavior_with_name('capture')):
            self.add_subbehavior(capture, 'capture', required = False, priority=5)

    def on_exit_capture (self):
        self.remove_all_subbehaviors()

    def on_enter_running(self):
        our_free_kick.OurFreeKick.Running = False

    def on_exit_running(self):
        our_free_kick.OurFreeKick.Running = False

    def must_shoot_time(self):
        return time.time() - self.start_time > self.shoot_time

    def is_goalie_close(self):
        # if any of there robots are in near chip range chip over them
        close_check = False
        for r in main.their_robots():
            close_check = close_check or (r.pos - main.ball().pos).mag() < self.goalie_range
        return close_check