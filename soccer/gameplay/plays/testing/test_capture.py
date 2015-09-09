import play
import behavior
import skills.move
import skills.capture
import single_robot_composite_behavior
import enum
import robocup

class Capturer(single_robot_composite_behavior.SingleRobotCompositeBehavior):

    class State(enum.Enum):
        setup = 1
        capturing = 2

    def __init__(self):
        super().__init__(continuous=True)
        self.add_state(Capturer.State.setup, behavior.Behavior.State.running)
        self.add_state(Capturer.State.capturing, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            Capturer.State.setup,
            lambda: True,
            'immediately')

        self.add_transition(Capturer.State.setup,
            Capturer.State.capturing,
            lambda: self.subbehavior_with_name('move').state == behavior.Behavior.State.completed,
            'robot away from ball')

        self.add_transition(Capturer.State.capturing,
            Capturer.State.setup,
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


# this test repeatedly runs the capture behavior
class TestCapture(play.Play):

    def __init__(self):
        super().__init__(continuous=True)

        self.add_subbehavior(Capturer(), 'capturer', required=True)

