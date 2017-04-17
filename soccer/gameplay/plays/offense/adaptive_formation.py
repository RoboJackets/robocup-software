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
import skills.move
import skills.capture


class AdaptiveFormation(standard_play.StandardPlay):
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

        # Min score to pass
        self.dribble_to_pass_cutoff = 0.1
        # Min score to shoot
        self.dribble_to_shoot_cutoff = 0.07
        # Dribbling skill
        self.dribbler = None
        # Dribble start point
        self.dribble_start_pt = robocup.Point(0, 0)
        # Max dribble distance per the rules with 10% wiggle room
        self.max_dribble_dist = 1 * .9
        # Kicker for a shot
        self.kick = None

        # Min field Y to clear
        self.clear_field_cutoff = constants.Field.Length * .2
        # Min dist to opponent before clear
        self.clear_distance_cutoff = constants.Field.Length * .1

        # The minimum increase from one cycle to the next to hold off Passing/Shooting/Clearing
        self.IncreasingChancesCutoff = 0.05
        # State Decision Variables
        self.shot_chance = 0
        self.pass_score = 0
        # Prev State Decision Variables
        self.prev_shot_chance = 0
        self.prev_pass_score = 0
        # Used to keep dribble within rules
        self.check_dribbling_timer = 0
        self.check_dribbling_timer_cutoff = 100

        # [type_]field_pos_weights: (Centerness, Distance to their goal, Angle off their goal)
        # [type]_weights: (space, field_position, shot_chance, kick_proximity)

        # Weights for the general field positioning
        self.field_pos_weights = (0.01, 3, 0.02)
        # Weights for finding best pass
        self.passing_weights = (2, 2, 15, 1)
        # Weights for finding where to dribble to
        self.dribbling_weights = (4, 2, 15, 1)
        # Weights to find where to chip the ball
        self.chip_field_pos_weights = (0.1, .2, 0.02)
        self.chip_pass_weights = (2, 10, 0, 10)

        self.kick_eval = robocup.KickEvaluator(main.system_state())

        # Add transitions
        self.add_transition(behavior.Behavior.State.start,
                            AdaptiveFormation.State.collecting, lambda: True,
                            'immediately')

        self.add_transition(
            AdaptiveFormation.State.collecting,
            AdaptiveFormation.State.dribbling,
            lambda: self.subbehavior_with_name('defend').state == behavior.Behavior.State.completed,
            'Ball Collected')

        self.add_transition(
            AdaptiveFormation.State.dribbling, AdaptiveFormation.State.passing,
            lambda: self.should_pass_from_dribble() and not self.should_shoot_from_dribble(),
            'Passing')

        self.add_transition(AdaptiveFormation.State.dribbling,
                            AdaptiveFormation.State.shooting,
                            lambda: self.should_shoot_from_dribble(),
                            'Shooting')

        self.add_transition(AdaptiveFormation.State.dribbling,
                            AdaptiveFormation.State.clearing,
                            lambda: self.should_clear_from_dribble(),
                            'Clearing')

        self.add_transition(
            AdaptiveFormation.State.passing, AdaptiveFormation.State.dribbling,
            lambda: self.subbehavior_with_name('pass').state == behavior.Behavior.State.completed,
            'Passed')

        # Reset to collecting when ball is lost at any stage
        self.add_transition(AdaptiveFormation.State.dribbling,
                            AdaptiveFormation.State.collecting,
                            lambda: False,  #evaluation.ball.robot_has_ball(self.dribbler.robot),
                            'Dribble: Ball Lost')
        self.add_transition(AdaptiveFormation.State.passing,
                            AdaptiveFormation.State.collecting,
                            lambda: self.subbehavior_with_name('pass').state == behavior.Behavior.State.cancelled or \
                                    self.subbehavior_with_name('pass').state == behavior.Behavior.State.failed,
                            'Passing: Ball Lost')
        self.add_transition(
            AdaptiveFormation.State.shooting,
            AdaptiveFormation.State.collecting,
            lambda: self.subbehavior_with_name('kick').is_done_running(),
            'Shooting: Ball Lost / Shot')
        self.add_transition(
            AdaptiveFormation.State.clearing,
            AdaptiveFormation.State.collecting,
            lambda: self.subbehavior_with_name('clear').is_done_running(),
            'Clearing: Ball Lost')

    def should_pass_from_dribble(self):

        # If pass is above cutoff and we dont have a good shot
        if (self.pass_score > self.dribble_to_pass_cutoff and \
            self.shot_chance < self.dribble_to_shoot_cutoff):
            print("Pass : " + str(self.pass_score) + " Shot : " + str(
                self.shot_chance))
            return True

        # Force pass if we are near our max dribble dist
        dribble_dist = (main.ball().pos - self.dribble_start_pt).mag()
        if (dribble_dist > self.max_dribble_dist):
            return True

        # Under cutoff
        return False

    def should_shoot_from_dribble(self):

        # If shot chance is improving significantly, hold off a second
        if (self.prev_shot_chance + self.IncreasingChancesCutoff <
                self.shot_chance):
            return False

        # Not in front of the half
        if (main.ball().pos.y < constants.Field.Length / 2):
            return False

        # If shot is above cutoff
        if (self.shot_chance > self.dribble_to_shoot_cutoff):
            print("Pass : " + str(self.pass_score) + " Shot : " + str(
                self.shot_chance))
            return True

        # Decreasing and under cutoff
        return False

    def should_clear_from_dribble(self):
        # If outside clear zone
        if (self.dribbler.pos.y > self.clear_field_cutoff):
            return False

        # If pass chances are getting better, hold off
        if (self.prev_pass_score + IncreasingChancesCutoff < self.pass_score):
            return False

        # TODO: See if there is space to dribble
        closest_distance = (evaluation.opponent.get_closest_opponent(
            main.ball().pos, 1).pos - main.ball().pos).mag()
        if (closest_distance > self.clear_distance_cutoff):
            return False

        return True

    def on_enter_collecting(self):
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
            main.ball().pos, None, main.our_robots(), self.field_pos_weights,
            self.dribbling_weights, False)
        self.add_subbehavior(self.dribbler, 'dribble', required=True)

        self.check_dribbling_timer = 0

    def execute_dribbling(self):
        # Find the closest bot weighting the ones in front of the ball more
        closest_bot = evaluation.opponent.get_closest_opponent(main.ball().pos,
                                                               0.9)

        # Grab best pass
        self.pass_target, self.pass_score = evaluation.passing_positioning.eval_best_receive_point(
            main.ball().pos, None, main.our_robots(), self.field_pos_weights,
            self.passing_weights, False)

        # Grab shot chance
        self.shot_chance = evaluation.shooting.eval_shot(main.ball().pos)

        # Recalculate dribbler pos
        self.check_dribbling_timer += 1
        if (self.check_dribbling_timer > self.check_dribbling_timer_cutoff):
            self.check_dribbling_timer = 0
            self.dribbler.pos, _ = evaluation.passing_positioning.eval_best_receive_point(
                main.ball().pos, None, main.our_robots(),
                self.field_pos_weights, self.dribbling_weights, False)

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
        self.remove_all_subbehaviors()

    def on_enter_shooting(self):
        # TODO: Use moving kick when completed
        self.kick = skills.pivot_kick.PivotKick()
        self.kick.target = constants.Field.TheirGoalSegment
        self.kick.aim_params['desperate_timeout'] = 3
        self.add_subbehavior(self.kick, 'kick', required=False)

    def on_exit_shooting(self):
        self.remove_all_subbehaviors()
        self.kick = None

    def on_enter_clearing(self):
        # Line kick with chip
        # Choose most open area / Best pass, weight forward
        # Decrease weight on sides of field due to complexity of settling
        self.pass_target, self.pass_score = evaluation.passing_positioning.eval_best_receive_point(
            main.ball().pos, None, main.our_robots(), self.field_pos_weights,
            self.passing_weights, False)

        clear = skills.pivot_kick.PivotKick()
        clear.target = self.pass_target
        clear.aim_params['desperate_timeout'] = 3
        clear.use_chipper = True
        self.add_subbehavior(chip, 'clear', required=False)

    def on_exit_clearing(self):
        self.remove_all_subbehaviors()

    def on_enter_passing(self):
        # TODO: Use the moving recieve when finished
        self.add_subbehavior(
            tactics.coordinated_pass.CoordinatedPass(self.pass_target), 'pass')

    def on_exit_passing(self):
        self.remove_all_subbehaviors()
