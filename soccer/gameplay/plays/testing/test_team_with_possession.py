import robocup
import standard_play
import behavior
import constants
import main
import skills.move
import skills.capture
import enum
import evaluation


class TestPossession(standard_play.StandardPlay):
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

        self.add_state(TestPossession.State.collecting,
                       behavior.Behavior.State.running)
        self.add_state(TestPossession.State.dribbling,
                       behavior.Behavior.State.running)
        self.add_state(TestPossession.State.shooting,
                       behavior.Behavior.State.running)
        self.add_state(TestPossession.State.clearing,
                       behavior.Behavior.State.running)
        self.add_state(TestPossession.State.passInMotion,
                       behavior.Behavior.State.running)
        self.add_state(TestPossession.State.passCollecting,
                       behavior.Behavior.State.running)
        self.add_state(TestPossession.State.oneTouch,
                       behavior.Behavior.State.running)

        # Add transitions
        self.add_transition(behavior.Behavior.State.start,
                            behavior.TestPossession.State.collecting,
                            lambda: True, 'immediately')

        self.add_transition(behavior.TestPossession.State.collecting,
                            behavior.TestPossession.State.dribbling,
                            lambda: False, 'Ball Collected')

        self.add_transition(behavior.TestPossession.State.dribbling,
                            behavior.TestPossession.State.passing,
                            lambda: False, 'Passing')

        self.add_transition(behavior.TestPossession.State.dribbling,
                            behavior.TestPossession.State.shooting,
                            lambda: False, 'Shooting')

        self.add_transition(behavior.TestPossession.State.dribbling,
                            behavior.TestPossession.State.clearing,
                            lambda: False, 'Clearing')

        # Passing states
        self.add_transition(behavior.TestPossession.State.passing,
                            behavior.TestPossession.State.passInMotion,
                            lambda: False, 'Pass Kicked')

        self.add_transition(behavior.TestPossession.State.passInMotion,
                            behavior.TestPossession.State.passCollecting,
                            lambda: False, 'Pass About to be Collected')

        self.add_transition(behavior.TestPossession.State.passCollecting,
                            behavior.TestPossession.State.oneTouch,
                            lambda: False, 'One Touch Shot')

        self.add_transition(behavior.TestPossession.State.passCollecting,
                            behavior.TestPossession.State.dribbling,
                            lambda: False, 'Pass Settled')

        # Reset to collecting when ball is lost at any stage
        self.add_transition(behavior.TestPossession.State.dribbling,
                            behavior.TestPossession.State.collecting,
                            lambda: False, 'Dribble: Ball Lost')

        self.add_transition(behavior.TestPossession.State.passing,
                            behavior.TestPossession.State.collecting,
                            lambda: False, 'Passing: Ball Lost')

        self.add_transition(behavior.TestPossession.State.shooting,
                            behavior.TestPossession.State.collecting,
                            lambda: False, 'Shooting: Ball Lost / Shot')

        self.add_transition(behavior.TestPossession.State.clearing,
                            behavior.TestPossession.State.collecting,
                            lambda: False, 'Clearing: Ball Lost')

        self.add_transition(behavior.TestPossession.State.passInMotion,
                            behavior.TestPossession.State.collecting,
                            lambda: False, 'PassInMotion: Ball Lost')

        self.add_transition(behavior.TestPossession.State.passCollecting,
                            behavior.TestPossession.State.collecting,
                            lambda: False, 'PassCollecting: Ball Lost')

        self.add_transition(behavior.TestPossession.State.oneTouch,
                            behavior.TestPossession.State.collecting,
                            lambda: False, 'OneTouch: Ball Lost / Shot')

    def on_enter_collecting(self):
        capture = skills.capture.Capture()
        self.add_subbehavior(capture, 'capture', required=True)

    def on_exit_collecting(self):
        self.remove_all_subbehaviors()
