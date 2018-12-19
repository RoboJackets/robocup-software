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


class AdaptiveFormation(standard_play.StandardPlay):

    # Min score to pass
    DRIBBLE_TO_PASS_CUTOFF = 0.1
    # Min score to shoot
    DRIBBLE_TO_SHOOT_CUTOFF = 0.07
    # Max dribble distance per the rules with 10% wiggle room
    MAX_DRIBBLE_DIST = 1 * .9

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
        # Shoot when chances are high
        shooting = 4
        # Clear when pass / dribble is worse and we are in our own zone
        clearing = 5

    def __init__(self):
        super().__init__(continuous=False)

        for s in AdaptiveFormation.State:
            self.add_state(s, behavior.Behavior.State.running)

        # Dribbling skill
        self.dribbler = None
        # Dribble start point
        self.dribble_start_pt = robocup.Point(0, 0)

        # Kicker for a shot
        self.kick = None
        # Controls robots while passes are being set up
        self.midfielders = None

        # State Decision Variables
        self.shot_chance = 0
        self.pass_score = 0
        # Prev State Decision Variables
        self.prev_shot_chance = 0
        self.prev_pass_score = 0
        # Used to keep dribble within rules
        self.check_dribbling_timer = 0
        self.check_dribbling_timer_cutoff = 100

        self.kick_eval = robocup.KickEvaluator(main.system_state())

        for r in main.our_robots():
            self.kick_eval.add_excluded_robot(r)

        # Add transitions
        self.add_transition(behavior.Behavior.State.start,
                            AdaptiveFormation.State.collecting, lambda: True,
                            'immediately')

        self.add_transition(AdaptiveFormation.State.collecting,
                            AdaptiveFormation.State.dribbling, lambda: self.
                            subbehavior_with_name('defend').state == behavior.
                            Behavior.State.completed, 'Ball Collected')

        self.add_transition(
            AdaptiveFormation.State.dribbling, AdaptiveFormation.State.passing,
            lambda: self.dribbler_has_ball() and self.should_pass_from_dribble(
            ) and not self.should_shoot_from_dribble(), 'Passing')

        self.add_transition(
            AdaptiveFormation.State.dribbling,
            AdaptiveFormation.State.shooting, lambda: self.dribbler_has_ball(
            ) and self.should_shoot_from_dribble(), 'Shooting')

        self.add_transition(
            AdaptiveFormation.State.dribbling,
            AdaptiveFormation.State.clearing, lambda: self.dribbler_has_ball(
            ) and self.should_clear_from_dribble(
            ) and not self.should_pass_from_dribble(
            ) and not self.should_shoot_from_dribble(), 'Clearing')

        self.add_transition(
            AdaptiveFormation.State.passing, AdaptiveFormation.State.dribbling,
            lambda: self.subbehavior_with_name(
                'pass').state == behavior.Behavior.State.completed, 'Passed')

        # Reset to collecting when ball is lost at any stage
        self.add_transition(AdaptiveFormation.State.dribbling,
                            AdaptiveFormation.State.collecting, lambda:
                            not self.dribbler_has_ball(), 'Dribble: Ball Lost')
        self.add_transition(
            AdaptiveFormation.State.passing,
            AdaptiveFormation.State.collecting, lambda: self.
            subbehavior_with_name('pass').state == behavior.Behavior.State.
            cancelled or self.subbehavior_with_name('pass').state == behavior.
            Behavior.State.failed, 'Passing: Ball Lost')
        self.add_transition(AdaptiveFormation.State.shooting,
                            AdaptiveFormation.State.collecting, lambda: self.
                            subbehavior_with_name('kick').is_done_running(),
                            'Shooting: Ball Lost / Shot')
        self.add_transition(AdaptiveFormation.State.clearing,
                            AdaptiveFormation.State.collecting, lambda: self.
                            subbehavior_with_name('clear').is_done_running(),
                            'Clearing: Ball Lost')

    @classmethod
    def score(cls):
        if (not main.game_state().is_playing()):
            return float("inf")
        if len(main.our_robots()) < 5:
            return float("inf")
        return 10

    def should_pass_from_dribble(self):

        # If pass is above cutoff and we dont have a good shot
        if (self.pass_score > AdaptiveFormation.DRIBBLE_TO_PASS_CUTOFF and
                self.shot_chance < AdaptiveFormation.DRIBBLE_TO_SHOOT_CUTOFF):
            print("Pass : " + str(self.pass_score) + " Shot : " + str(
                self.shot_chance))
            return True

        # Force pass if we are near our max dribble dist
        dribble_dist = (main.ball().pos - self.dribble_start_pt).mag()
        if (dribble_dist > AdaptiveFormation.MAX_DRIBBLE_DIST):
            return True

        # Under cutoff
        return False

    def should_shoot_from_dribble(self):

        # If shot chance is improving significantly, hold off a second
        if (self.prev_shot_chance + AdaptiveFormation.INCREASING_CHANCE_CUTOFF
                < self.shot_chance):
            return False

        # Not in front of the half
        if (main.ball().pos.y < constants.Field.Length / 2):
            return False

        # If shot is above cutoff
        if (self.shot_chance > AdaptiveFormation.DRIBBLE_TO_SHOOT_CUTOFF):
            print("Pass : " + str(self.pass_score) + " Shot : " + str(
                self.shot_chance))
            return True

        # Decreasing and under cutoff
        return False

    def should_clear_from_dribble(self):
        # If outside clear zone
        if (self.dribbler.pos.y > AdaptiveFormation.CLEAR_FIELD_CUTOFF):
            return False

        # If pass chances are getting better, hold off
        if (self.prev_pass_score + AdaptiveFormation.INCREASING_CHANCE_CUTOFF <
                self.pass_score):
            return False

        # TODO: See if there is space to dribble
        closest_distance = (evaluation.opponent.get_closest_opponent(
            main.ball().pos, 1).pos - main.ball().pos).mag()
        if (closest_distance > AdaptiveFormation.CLEAR_DISTANCE_CUTOFF):
            return False

        return True

    def dribbler_has_ball(self):
        return any(
            evaluation.ball.robot_has_ball(r) for r in main.our_robots())

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
            main.our_robots(), AdaptiveFormation.FIELD_POS_WEIGHTS,
            AdaptiveFormation.NELDER_MEAD_ARGS,
            AdaptiveFormation.DRIBBLING_WEIGHTS)

        self.add_subbehavior(self.dribbler, 'dribble', required=True)

        self.check_dribbling_timer = 0

        if (not self.has_subbehavior_with_name('midfielders')):
            self.midfielders = tactics.advance_zone_midfielder.AdvanceZoneMidfielder(
            )
            self.add_subbehavior(
                self.midfielders, 'midfielders', required=False, priority=10)
        self.midfielders.kick = False

    def execute_dribbling(self):
        # Grab best pass
        self.pass_target, self.pass_score = evaluation.passing_positioning.eval_best_receive_point(
            main.ball().pos,
            main.our_robots(), AdaptiveFormation.FIELD_POS_WEIGHTS,
            AdaptiveFormation.NELDER_MEAD_ARGS,
            AdaptiveFormation.PASSING_WEIGHTS)

        self.midfielders.kick = False
        self.midfielders.passing_point = self.pass_target

        # Grab shot chance
        self.shot_chance = evaluation.shooting.eval_shot(main.ball().pos)

        # Recalculate dribbler pos
        self.check_dribbling_timer += 1
        if (self.check_dribbling_timer > self.check_dribbling_timer_cutoff):
            self.check_dribbling_timer = 0
            self.dribbler.pos, _ = evaluation.passing_positioning.eval_best_receive_point(
                main.ball().pos,
                main.our_robots(), AdaptiveFormation.FIELD_POS_WEIGHTS,
                AdaptiveFormation.NELDER_MEAD_ARGS,
                AdaptiveFormation.DRIBBLING_WEIGHTS)

        # TODO: Get list of top X pass positions and have robots in good positions to reach them
        # Good positions can be definied by offensive / defensive costs
        # Offensive positions move onto the ball in the direction of the goal
        # Defensive cover the center of the field

        # Setup previous values (Basic complementary filter)
        c = .8
        self.prev_shot_chance = c * self.shot_chance + \
                                (1 - c) * self.prev_shot_chance
        self.prev_pass_score = c * self.pass_score + \
                               (1 - c) * self.prev_pass_score

    def on_exit_dribbling(self):
        self.remove_subbehavior('dribble')

    def on_enter_shooting(self):
        # TODO: Use moving kick when completed
        self.kick = skills.pivot_kick.PivotKick()
        self.kick.target = constants.Field.TheirGoalSegment
        self.kick.aim_params['desperate_timeout'] = 3
        self.midfielders.kick = True
        self.add_subbehavior(self.kick, 'kick', required=False)

    def on_exit_shooting(self):
        self.remove_subbehavior('kick')
        self.kick = None
        self.midfielders.kick = False

    def on_enter_clearing(self):
        # Line kick with chip
        # Choose most open area / Best pass, weight forward
        # Decrease weight on sides of field due to complexity of settling
        self.pass_target, self.pass_score = evaluation.passing_positioning.eval_best_receive_point(
            main.ball().pos,
            main.our_robots(), AdaptiveFormation.FIELD_POS_WEIGHTS,
            AdaptiveFormation.NELDER_MEAD_ARGS,
            AdaptiveFormation.PASSING_WEIGHTS)

        clear = skills.pivot_kick.PivotKick()
        clear.target = self.pass_target
        clear.aim_params['desperate_timeout'] = 3
        clear.use_chipper = True
        self.add_subbehavior(clear, 'clear', required=False)

    def on_exit_clearing(self):
        self.remove_subbehavior('clear')

    def on_enter_passing(self):
        # TODO: Use the moving recieve when finished
        self.midfielders.kick = False
        self.add_subbehavior(
            tactics.coordinated_pass.CoordinatedPass(self.pass_target), 'pass')

    def on_exit_passing(self):
        self.remove_subbehavior('pass')
