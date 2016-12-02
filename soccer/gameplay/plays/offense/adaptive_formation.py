import robocup
import standard_play
import behavior
import constants
import main
import skills.move
import skills.capture
import enum
import evaluation.passing_positioning
import tactics.coordinated_pass

# TODO: Copy-Paste notes document into here

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

        # Pass is in motion, move to collect pass
        passInMotion = 6
        # Check if one touch goal is better than settling
        passCollecting = 7
        # One touch shot
        oneTouch = 8

    def __init__(self):
        super().__init__(continuous=False)

        for s in AdaptiveFormation.State:
            self.add_state(s, behavior.Behavior.State.running)

        # Min score to pass
        self.dribbleToPassCutoff = 0.3
        # Min score to shoot
        self.dribbleToShootCutoff = 0.2

        # Min field Y to clear
        self.clearFieldCutoff = constants.Field.Length / 5 # in our 20%

        # Min onetouch score
        self.oneTouchShotCutoff = 0.9

        # Min distance to switch phases
        self.passCollectingDist = 0.5

        # The minimum increase from one cycle to the next to hold off Passing/Shooting/Clearing
        self.IncreasingChancesCutoff = 0.001

        # Add transitions
        self.add_transition(behavior.Behavior.State.start,
                            AdaptiveFormation.State.collecting, 
                            lambda: True,
                            'immediately')

        self.add_transition(AdaptiveFormation.State.collecting,
                            AdaptiveFormation.State.dribbling, 
                            lambda: self.subbehavior_with_name('capture').is_done_running(),
                            'Ball Collected')

        self.add_transition(AdaptiveFormation.State.dribbling,
                            AdaptiveFormation.State.passing, 
                            lambda: self.should_pass_from_dribble(),
                            'Passing')

        self.add_transition(AdaptiveFormation.State.dribbling,
                            AdaptiveFormation.State.shooting, 
                            lambda: self.should_shoot_from_dribble(),
                            'Shooting')

        self.add_transition(AdaptiveFormation.State.dribbling,
                            AdaptiveFormation.State.clearing, 
                            lambda: self.should_clear_from_dribble(),
                            'Clearing')

        # Passing states
        self.add_transition(AdaptiveFormation.State.passing,
                            AdaptiveFormation.State.passInMotion, 
                            lambda: self.subbehavior_with_name('pass').state == behavior.Behavior.State.completed,
                            'Pass Kicked')

        self.add_transition(AdaptiveFormation.State.passInMotion,
                            AdaptiveFormation.State.passCollecting, 
                            lambda: True,
                            'Pass About to be Collected')

        self.add_transition(AdaptiveFormation.State.passCollecting,
                            AdaptiveFormation.State.oneTouch, 
                            lambda: False,
                            'One Touch Shot')

        self.add_transition(AdaptiveFormation.State.passCollecting,
                            AdaptiveFormation.State.dribbling, 
                            lambda: True,
                            'Pass Settled')

        # Reset to collecting when ball is lost at any stage
        self.add_transition(AdaptiveFormation.State.dribbling,
                            AdaptiveFormation.State.collecting, 
                            lambda: False,
                            'Dribble: Ball Lost')
        self.add_transition(AdaptiveFormation.State.passing,
                            AdaptiveFormation.State.collecting, 
                            lambda: False,
                            'Passing: Ball Lost')
        self.add_transition(AdaptiveFormation.State.shooting,
                            AdaptiveFormation.State.collecting, 
                            lambda: self.subbehavior_with_name('kick').state == behavior.Behavior.State.completed,
                            'Shooting: Ball Lost / Shot')
        self.add_transition(AdaptiveFormation.State.clearing,
                            AdaptiveFormation.State.collecting, 
                            lambda: False,
                            'Clearing: Ball Lost')
        self.add_transition(AdaptiveFormation.State.passInMotion,
                            AdaptiveFormation.State.collecting, 
                            lambda: False,
                            'PassInMotion: Ball Lost')
        self.add_transition(AdaptiveFormation.State.passCollecting,
                            AdaptiveFormation.State.collecting, 
                            lambda: False,
                            'PassCollecting: Ball Lost')
        self.add_transition(AdaptiveFormation.State.oneTouch,
                            AdaptiveFormation.State.collecting, 
                            lambda: False,
                            'OneTouch: Ball Lost / Shot')

        self.win_eval = robocup.WindowEvaluator(main.system_state())
        self.dribbler = None
        
        # State Transition Variable
        self.shot_location = None
        self.pass_pos = None

        # State Deciding Variables
        self.shot_chance = 0
        self.pass_score = 0
        
        # Prev State Deciding Variables
        self.prev_shot_chance = 0
        self.prev_pass_score = 0

    def should_pass_from_dribble(self):
        # If the pass chances are getting better, hold off
        if (self.prev_pass_score + self.IncreasingChancesCutoff < self.pass_score):
            return False

        # If pass is above cutoff
        if (self.pass_score > self.dribbleToPassCutoff):
            return True

        # Decreasing and under cutoff
        return False
    def should_shoot_from_dribble(self):
        # If the shot chances are getting better, hold off
        if (self.prev_shot_chance + self.IncreasingChancesCutoff < self.shot_chance):
            return False

        # If shot is above cutoff
        if (self.shot_chance > self.dribbleToShootCutoff):
            return True

        # Decreasing and under cutoff
        return False
    # TODO: See if there is space to dribble
    def should_clear_from_dribble(self):
        # If outside zone of goal
        if (self.dribbler.pos.y > self.clearFieldCutoff):
            return False

        # If pass chances are getting better, hold off
        if (self.prev_pass_score + IncreasingChancesCutoff < self.pass_score):
            return False

        return True

    def on_enter_collecting(self):
        capture = skills.capture.Capture()
        self.add_subbehavior(capture, 'capture', required=True)

    def on_exit_collecting(self):
        self.remove_all_subbehaviors()

    def on_enter_dribbling(self):
        self.dribbler = skills.dribble.Dribble()
        # TODO: Make the dribble location better
        self.dribbler.pos = robocup.Point(0,constants.Field.Length)
        self.add_subbehavior(self.dribbler, 'dribble', required=True)

    def execute_dribbling(self):
        # Setup previous values (Basic complementary filter)
        c = 0.7
        self.prev_shot_chance = c*self.shot_chance + (1-c)*self.prev_shot_chance
        self.prev_pass_score = c*self.pass_score + (1-c)*self.prev_pass_score

        # Find closest bot weighting the ones in front higher
        closest_bot = evaluation.opponent.get_closest_opponent(main.ball().pos, 0.9)
        
        # Grab best pass
        self.pass_pos, self.pass_score = evaluation.passing_positioning.eval_best_receive_point(
                                                    main.ball().pos, None, main.our_robots(),
                                                    (0.01, 3, 0.02), (2, 2, 15), True)

        # Grab shot chance
        self.shot_chance = evaluation.shooting.eval_shot(main.ball().pos)

        # Add some sort of dibble reset for the 1000cm limit in the rules

    def on_exit_dribbling(self):
        self.remove_all_subbehaviors()

    def on_enter_shooting(self):
        kick = skills.pivot_kick.PivotKick()
        kick.target = constants.Field.TheirGoalSegment
        kick.aim_params['desperate_timeout'] = 3
        self.add_subbehavior(kick, 'kick', required=False)

    def on_exit_shooting(self):
        self.remove_all_subbehaviors()

    def on_enter_passing(self):
        # Setup passer
        # Setup reciever
        self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(self.pass_pos), 'pass')

    def execute_passing(self):
        # Wait until the reciever will be able to get there in time
        # Then kick the ball
        pass

    def on_exit_passing(self):
        self.remove_all_subbehaviors()

    def on_enter_clearing(self):
        # Line kick with chip
        # Choose most open area / Best pass, weight forward
        pass

    def execute_clearing(self):
        pass

    def on_exit_passing(self):
        self.remove_all_subbehaviors()