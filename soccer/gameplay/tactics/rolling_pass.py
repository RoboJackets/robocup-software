import robocup
import play
import behavior
import constants
import main
import skills.move
import skills.capture
import enum
import evaluation
import tactics.coordinated_pass
import skills.angle_receive


class RollingPass(play.Play):
    #INCOMPLETE
    #Just an idea for a tactic I wanted to test

    class State(enum.Enum):
        ## Move A and move B, capture in setup
        capturing = 1
        ## Pick best target, add coordinated pass subbehavior
        passing = 2

    def __init__(self):
        super().__init__(continuous=False)

        self._pos= robocup.point(0,7.5)
        self._threshold=.5

        self.add_state(RollingPass.State.capturing,
                       behavior.Behavior.State.running)
        self.add_state(RollingPass.State.passing,
                       behavior.Behavior.State.running)

        # Add transitions
        self.add_transition(behavior.Behavior.State.start,
                            RollingPass.State.capture, lambda: True,
                            'immediately')
        self.add_transition(
            RollingPass.State.capturing, RollingPass.State.passing,
            lambda: self.all_subbehaviors_completed(),
            'all subbehaviors completed')

        self.add_transition(
            RollingPass.State.passing, RollingPass.State.passing,
            lambda: self.subbehavior_with_name('recieve').State=behavior.Behavior.State.completed)

        self.add_transition(
            RollingPass.State.passing, behavior.Behavior.State.completed,
            lambda: False,
            'Robot in final position')

        self.passRobot1 = None
        self.passRobot2 = None


    @property
    def pos(self):
        return self._pos

    @pos.setter
    def pos(self, value):
        self._pos = value

    @property
    def pos(self):
        return self._threshold

    @pos.setter
    def pos(self, value):
        self._threshold = value


    def on_enter_capturing(self):
        capture = skills.capture.Capture()
        self.add_subbehavior(
            skills.move.Move(), #pos
            'moveB',
            required=True)
        self.add_subbehavior(capture, 'capture', required=True)
    
    @classmethod
    def score(cls):
        if main.game_state().is_playing():
            return 9
        return float("inf")
