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

    BumpKickPower = 0.01
    FullKickPower = 1
    MaxShootingAngle = 80

    class State(Enum):
        dribbling = 0
        chipping = 1
        shooting = 2
        capture = 3

    def __init__(self):
        super().__init__(continuous=True)

        for s in OurShootoutChip.State :
            self.add_state(s, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            OurShootoutChip.State.dribbling, lambda: True,
                            'immediately')

        self.add_transition(OurShootoutChip.State.dribbling,
                            OurShootoutChip.State.chipping, lambda: self.is_goalie_close(),
                            'goalie in range')

        self.add_transition(OurShootoutChip.State.dribbling,
                            OurShootoutChip.State.shooting,
                            lambda: not self.is_goalie_close() and self.must_shoot_time(),
                            'must shoot')

        self.add_transition(OurShootoutChip.State.chipping, 
                            OurShootoutChip.State.capture,
                            lambda: self.subbehavior_with_name('chipping').state == behavior.Behavior.State.completed,
                            'recapture')

        self.add_transition(OurShootoutChip.State.capture,
                            OurShootoutChip.State.shooting,
                            lambda:self.subbehavior_with_name('capture').state == behavior.Behavior.State.completed,
                            'reshoot')

        self.total_time = 10.0
        self.shoot_time = 6.0
        self.start_time = time.time()
        self.chip_distance = 1.5
        self.goalie_range = 2

        # grab the ball and charge the goal

        # if they charge us wait for them to get within the free kick hold range and chip it over them
        # if they dont charge us continue driving until we hit the 1 meter range limit and then kick into the goal

    @classmethod
    def handles_goalie(cls):
        return True

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0# if gs.is_penalty_shootout() and gs.is_our_penalty() else float("inf")

    @classmethod
    def is_restart(cls):
        return False

    def on_enter_dribbling(self):
        robot_pos = main.our_robots()[0].pos
        center_goal = constants.Field.TheirGoalSegment.center()
        dribble_point = (center_goal - robot_pos).normalized() + robot_pos
        print(dribble_point)
        dribble = skills.dribble.Dribble(dribble_point)
        if (not self.has_subbehavior_with_name('dribble')):
            self.add_subbehavior(dribble, 'dribble', required = False, priority = 5)

    def on_exit_dribbling(self):
        self.remove_all_subbehaviors()

    def execute_chipping(self):
            #self.remove_subbehavior('kicker')
            kicker = skills.line_kick.LineKick()
            kicker.use_chipper = True
            kicker.min_chip_range = our_free_kick.OurFreeKick.MinChipRange
            kicker.max_chip_range = our_free_kick.OurFreeKick.MaxChipRange
            gap = evaluation.shooting.find_gap(max_shooting_angle= our_free_kick.OurFreeKick.MaxShootingAngle)
            kicker.target = gap
            if (not self.has_subbehavior_with_name('chipper')) :
                self.add_subbehavior(kicker, 'chipper', required=False, priority=5)

    def on_exit_chipping(self):
        self.remove_all_subbehaviors()

    def execute_shooting(self):
        kicker = skills.line_kick.LineKick()
        kicker.target = constants.Field.TheirGoalSegment
        if (not self.has_subbehavior_with_name('kicker')) :
                self.add_subbehavior(kicker, 'kicker', required=False, priority=5)

    def on_exit_shooting(self):
        self.remove_all_subbehaviors()

    def execute_capture (self):
        capture = tactics.capture.Capture()
        if (not self.has_subbehavior_with_name('capture')):
            self.add_subbehavior(capture, 'capture', required = False, priority=5)

    def on_exit_capture (self):
        self.remove_all_subbehaviors()

    def on_enter_running(self):
        our_free_kick.OurFreeKick.Running = False

    #def execute_running(self):
    #    print(time.time())

    def on_exit_running(self):
        our_free_kick.OurFreeKick.Running = False

    def must_shoot_time(self):
        print(time.time() - self.start_time)
        return time.time() - self.start_time > self.shoot_time

    def is_goalie_close(self):
        close_check = False
        for r in main.their_robots():
            close_check = r or (r.pos - main.our_robots()[0]).mag() < self.goalie_range
