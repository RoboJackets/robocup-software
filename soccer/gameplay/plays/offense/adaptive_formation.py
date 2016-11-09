import robocup
import standard_play
import behavior
import constants
import main
import skills.move
import skills.capture
import enum
import evaluation

class AdaptiveFormation(standard_play.StandardPlay):
    class State(enum.Enum):
        # Collect the ball / Full court defense
        collecting = 1
        # Dribble for a second and prepare to pass / shoot / clear
        dribbling = 2
        # Pass when it is better than dribbling
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

        self.add_state(AdaptiveFormation.State.collecting,
                       behavior.Behavior.State.running)
        self.add_state(AdaptiveFormation.State.dribbling,
                       behavior.Behavior.State.running)
        self.add_state(AdaptiveFormation.State.shooting,
                       behavior.Behavior.State.running)
        self.add_state(AdaptiveFormation.State.clearing,
                       behavior.Behavior.State.running)
        self.add_state(AdaptiveFormation.State.passInMotion,
                       behavior.Behavior.State.running)
        self.add_state(AdaptiveFormation.State.passCollecting,
                       behavior.Behavior.State.running)
        self.add_state(AdaptiveFormation.State.oneTouch,
                       behavior.Behavior.State.running)


        self.dribbleToPassCutoff = 0.5
        
        self.dribbleToShootCutoff = 0.8

        self.clearFieldCutoff = 1
        self.dribbleToClearCutoff = 0.2

        self.oneTouchShotCutoff = 0.9

        self.passCollectingDist = 0.5

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
                            lambda: False,
                            'Passing')

        self.add_transition(AdaptiveFormation.State.dribbling,
                            AdaptiveFormation.State.shooting, 
                            lambda: False,
                            'Shooting')

        self.add_transition(AdaptiveFormation.State.dribbling,
                            AdaptiveFormation.State.clearing, 
                            lambda: False,
                            'Clearing')

        # Passing states
        self.add_transition(AdaptiveFormation.State.passing,
                            AdaptiveFormation.State.passInMotion, 
                            lambda: False,
                            'Pass Kicked')

        self.add_transition(AdaptiveFormation.State.passInMotion,
                            AdaptiveFormation.State.passCollecting, 
                            lambda: False,
                            'Pass About to be Collected')

        self.add_transition(AdaptiveFormation.State.passCollecting,
                            AdaptiveFormation.State.oneTouch, 
                            lambda: False,
                            'One Touch Shot')

        self.add_transition(AdaptiveFormation.State.passCollecting,
                            AdaptiveFormation.State.dribbling, 
                            lambda: False,
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
                            lambda: False,
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

        self.dribbler = None


    def on_enter_collecting(self):
        capture = skills.capture.Capture()
        self.add_subbehavior(capture, 'capture', required=True)

    def on_exit_collecting(self):
        self.remove_all_subbehaviors()

    def on_enter_dribbling(self):
        self.dribbler = skills.dribble.Dribble()
        self.dribbler.pos = robocup.Point(0,constants.Field.Length)
        self.add_subbehavior(self.dribbler, 'dribble', required=True)

    def execute_dribbling(self):
        # Get distance from nearest opp robot
        # Figure out where to pass to get best shot
        #   Grid?
        # Figure out if we should shoot
        # Figure out if we should chip
        pass