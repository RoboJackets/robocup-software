import play
import behavior
import skills.move
import skills.capture
import enum
import robocup


# this test repeatedly runs the capture behavior
class TestCapture(play.Play):

    class State(enum.Enum):
        setup = 1
        capturing = 2

    def __init__(self):
        super().__init__(continuous=True)

        self.add_state(TestCapture.State.setup, behavior.Behavior.State.running)
        self.add_state(TestCapture.State.capturing, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            TestCapture.State.setup,
            lambda: True,
            'immediately')

        self.add_transition(TestCapture.State.setup,
            TestCapture.State.capturing,
            lambda: self.subbehavior_with_name('move').state == behavior.Behavior.State.completed,
            'robot away from ball')

        self.add_transition(TestCapture.State.capturing,
            TestCapture.State.setup,
            lambda: self.subbehavior_with_name('capture').state == behavior.Behavior.State.completed,
            'successful capture')


    def on_enter_capturing(self):
        self.add_subbehavior(skills.capture.Capture(), 'capture', required=True)
    def on_exit_capturing(self):
        self.remove_subbehavior('capture')


    def on_enter_setup(self):
        m = skills.move.Move()
        m.pos = robocup.Point(0, 1.1)
        self.add_subbehavior(m, 'move', required=True)
    def on_exit_setup(self):
        self.remove_subbehavior('move')
