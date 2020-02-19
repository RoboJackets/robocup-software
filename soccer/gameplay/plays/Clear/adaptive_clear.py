import main
import robocup
import behavior
import constants
import enum

import standard_play
import evaluation.ball
import evaluation.passing_positioning
import tactics.coordinated_pass
import tactics.defensive_forward
import tactics.simple_zone_midfielder
import tactics.advance_zone_midfielder
import skills.move
import skills.capture
import situational_play_selection


##
# This is essentially a copy of adaptive formation, found in the legacy folder,
# With the shooting option removed. It is intended to be used as a clearing play
class AdaptiveClear(standard_play.StandardPlay):

    _situationList = [
        situational_play_selection.SituationalPlaySelector.Situation.CLEAR,
    ] # yapf: disable

    # Min score to pass
    DRIBBLE_TO_PASS_CUTOFF = 0.1
    # Max dribble distance per the rules with 10% wiggle room
    MAX_DRIBBLE_DIST = 1 * .9

    MIN_PASS_DIST = .2

    # Min field Y to clear
    CLEAR_FIELD_CUTOFF = constants.Field.Length * .2
    # Min dist to opponent before clear
    CLEAR_DISTANCE_CUTOFF = constants.Field.Length * .1

    # The minimum increase from one cycle to the next to hold off Passing/Shooting/Clearing
    INCREASING_CHANCE_CUTOFF = 0.05

    # [type_]FIELD_POS_WEIGHTS: (Centerness, Distance to their goal, Angle off their goal)
    # [type]_weights: (space, field_position, shot_chance, kick_proximity)

    # Weights for the general field positioning
    FIELD_POS_WEIGHTS = (0.01, 3, 0.02)
    # Weights for finding best pass
    PASSING_WEIGHTS = (2, 2, 15, 1)
    # Weights for finding where to dribble to
    DRIBBLING_WEIGHTS = (4, 2, 15, 1)
    # Weights to find where to chip the ball
    CHIP_FIELD_POS_WEIGHTS = (0.1, .2, 0.02)
    CHIP_PASS_WEIGHTS = (2, 10, 0, 10)

    # Initial arguements for the nelder mead optimization in passing positioning
    NELDER_MEAD_ARGS = (robocup.Point(0.5, 2), \
                        robocup.Point(0.01, 0.01), 1, 2, \
                        0.75, 0.5, 50, 1, 0.1)

    class State(enum.Enum):
        # Collect the ball / Full court defense
        collecting = 1
        # Dribble for a second and prepare to pass / shoot / clear
        dribbling = 2
        # Pass when someone is open
        passing = 3
        # Clear when pass / dribble is worse and we are in our own zone
        clearing = 4

    def __init__(self):
        super().__init__(continuous=False)

        for s in AdaptiveClear.State:
            self.add_state(s, behavior.Behavior.State.running)

        # Dribbling skill
        self.dribbler = None
        # Dribble start point
        self.dribble_start_pt = robocup.Point(0, 0)

        # Controls robots while passes are being set up
        self.midfielders = None

        # State Decision Variables
        self.pass_score = 0
        # Prev State Decision Variables
        self.prev_pass_score = 0
        # Used to keep dribble within rules
        self.check_dribbling_timer = 0
        self.check_dribbling_timer_cutoff = 100

        # Add transitions
        self.add_transition(behavior.Behavior.State.start,
                            AdaptiveClear.State.collecting, lambda: True,
                            'immediately')

        self.add_transition(
            AdaptiveClear.State.collecting,
            AdaptiveClear.State.dribbling, lambda: self.subbehavior_with_name(
                'defend').state == behavior.Behavior.State.completed,
            'Ball Collected')

        self.add_transition(
            AdaptiveClear.State.dribbling,
            AdaptiveClear.State.passing, lambda: self.dribbler_has_ball(
            ) and self.should_pass_from_dribble(), 'Passing')

        self.add_transition(
            AdaptiveClear.State.dribbling,
            AdaptiveClear.State.clearing, lambda: self.dribbler_has_ball(
            ) and self.should_clear_from_dribble(
            ) and not self.should_pass_from_dribble(), 'Clearing')

        self.add_transition(
            AdaptiveClear.State.passing,
            AdaptiveClear.State.dribbling, lambda: self.subbehavior_with_name(
                'pass').state == behavior.Behavior.State.completed, 'Passed')

        # Reset to collecting when ball is lost at any stage
        self.add_transition(AdaptiveClear.State.dribbling,
                            AdaptiveClear.State.collecting, lambda: not self.
                            dribbler_has_ball(), 'Dribble: Ball Lost')
        self.add_transition(
            AdaptiveClear.State.passing, AdaptiveClear.State.collecting,
            lambda: self.subbehavior_with_name('pass').state == behavior.
            Behavior.State.cancelled or self.subbehavior_with_name('pass').
            state == behavior.Behavior.State.failed, 'Passing: Ball Lost')

        self.add_transition(
            AdaptiveClear.State.clearing,
            AdaptiveClear.State.collecting, lambda: self.subbehavior_with_name(
                'clear').is_done_running(), 'Clearing: Ball Lost')

    @classmethod
    def score(cls):
        score = super().score()

        #If we get a valid score from the super function, then we should calculate
        #an offset and sum that with score and return that
        if (score != float("inf")):
            #currently the offset is just 0 because we haven't made one
            scoreOffset = 0
            return score + scoreOffset
        else:
            if (not main.game_state().is_playing()):
                return float("inf")
            if len(main.our_robots()) < 5:
                return float("inf")
            return 8

    def should_pass_from_dribble(self):

        # If pass is above cutoff
        if (self.pass_score > AdaptiveClear.DRIBBLE_TO_PASS_CUTOFF):
            print("Pass : " + str(self.pass_score))
            return True

        # Force pass if we are near our max dribble dist
        dribble_dist = (main.ball().pos - self.dribble_start_pt).mag()
        if (dribble_dist > AdaptiveClear.MAX_DRIBBLE_DIST):
            return True

        # Under cutoff
        return False

    def should_clear_from_dribble(self):
        # If outside clear zone
        if (self.dribbler.pos.y > AdaptiveClear.CLEAR_FIELD_CUTOFF):
            return False

        # If pass chances are getting better, hold off
        if (self.prev_pass_score + AdaptiveClear.INCREASING_CHANCE_CUTOFF <
                self.pass_score):
            return False

        # TODO: See if there is space to dribble
        closest_distance = (evaluation.opponent.get_closest_opponent(
            main.ball().pos, 1).pos - main.ball().pos).mag()
        if (closest_distance > AdaptiveClear.CLEAR_DISTANCE_CUTOFF):
            return False

        return True

    def dribbler_has_ball(self):
        return any(r.has_ball() for r in main.our_robots())

    def on_enter_collecting(self):
        self.remove_all_subbehaviors()

        # 2 man to man defenders and 1 zone defender
        defensive_forward = tactics.defensive_forward.DefensiveForward()
        self.add_subbehavior(defensive_forward, 'defend', required=True)

    def on_exit_collecting(self):
        self.remove_all_subbehaviors()

    def on_enter_dribbling(self):
        self.dribbler = skills.dribble.Dribble()
        self.dribble_start_pt = main.ball().pos

        # Dribbles toward the best receive point

        self.dribbler.pos, _ = evaluation.passing_positioning.eval_best_receive_point(
            main.ball().pos,
            main.our_robots(), AdaptiveClear.MIN_PASS_DIST,
            AdaptiveClear.FIELD_POS_WEIGHTS, AdaptiveClear.NELDER_MEAD_ARGS,
            AdaptiveClear.DRIBBLING_WEIGHTS)

        self.add_subbehavior(self.dribbler, 'dribble', required=True)

        self.check_dribbling_timer = 0

        if (not self.has_subbehavior_with_name('midfielders')):
            self.midfielders = tactics.simple_zone_midfielder.SimpleZoneMidfielder(
            )
            self.add_subbehavior(
                self.midfielders, 'midfielders', required=False, priority=10)

    def execute_dribbling(self):
        # Grab best pass
        self.pass_target, self.pass_score = evaluation.passing_positioning.eval_best_receive_point(
            main.ball().pos,
            main.our_robots(), AdaptiveClear.MIN_PASS_DIST,
            AdaptiveClear.FIELD_POS_WEIGHTS, AdaptiveClear.NELDER_MEAD_ARGS,
            AdaptiveClear.PASSING_WEIGHTS)

        # Recalculate dribbler pos
        self.check_dribbling_timer += 1
        if (self.check_dribbling_timer > self.check_dribbling_timer_cutoff):
            self.check_dribbling_timer = 0
            self.dribbler.pos, _ = evaluation.passing_positioning.eval_best_receive_point(
                main.ball().pos,
                main.our_robots(), AdaptiveClear.MIN_PASS_DIST,
                AdaptiveClear.FIELD_POS_WEIGHTS,
                AdaptiveClear.NELDER_MEAD_ARGS,
                AdaptiveClear.DRIBBLING_WEIGHTS)

        # TODO: Get list of top X pass positions and have robots in good positions to reach them
        # Good positions can be definied by offensive / defensive costs
        # Offensive positions move onto the ball in the direction of the goal
        # Defensive cover the center of the field

    def on_exit_dribbling(self):
        self.remove_subbehavior('dribble')

    def on_enter_clearing(self):
        # Line kick with chip
        # Choose most open area / Best pass, weight forward
        # Decrease weight on sides of field due to complexity of settling
        self.pass_target, self.pass_score = evaluation.passing_positioning.eval_best_receive_point(
            main.ball().pos,
            main.our_robots(), AdaptiveClear.FIELD_POS_WEIGHTS,
            AdaptiveClear.NELDER_MEAD_ARGS, AdaptiveClear.PASSING_WEIGHTS)

        clear = skills.pivot_kick.PivotKick()
        clear.target = self.pass_target
        clear.aim_params['desperate_timeout'] = 1
        clear.use_chipper = True
        self.add_subbehavior(clear, 'clear', required=False)

    def on_exit_clearing(self):
        self.remove_subbehavior('clear')

    def on_enter_passing(self):
        # TODO: Use the moving receive when finished
        self.add_subbehavior(
            tactics.coordinated_pass.CoordinatedPass(self.pass_target), 'pass')

    def on_exit_passing(self):
        self.remove_subbehavior('pass')
