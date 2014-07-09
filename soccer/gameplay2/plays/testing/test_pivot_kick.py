import play
import behavior
import skills.pivot_kick
import skills.move
import enum
import main
import constants
import robocup


# this test repeatedly runs the PivotKick behavior aimed at our goal
class TestPivotKick(play.Play):

    class State(enum.Enum):
        waiting = 1
        kicking = 2

    def __init__(self):
        super().__init__(continuous=True)

        self.add_state(TestPivotKick.State.waiting, behavior.Behavior.State.running)
        self.add_state(TestPivotKick.State.kicking, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            TestPivotKick.State.waiting,
            lambda: True,
            'immediately')

        self.add_transition(TestPivotKick.State.waiting,
            TestPivotKick.State.kicking,
            lambda: self.ball_kickable(),
            'robot away from ball')

        self.add_transition(TestPivotKick.State.kicking,
            TestPivotKick.State.waiting,
            lambda: self.subbehavior_with_name('kick').state == behavior.Behavior.State.completed,
            'successful kick')

        self.add_transition(TestPivotKick.State.kicking,
            TestPivotKick.State.waiting,
            lambda: not self.ball_kickable(),
            'ball not kickable')


    def ball_kickable(self):
        # print("kickable")
        return main.ball().valid and main.ball().pos.y > constants.Field.ArcRadius + constants.Robot.Radius * 1.5


    def on_enter_kicking(self):
        kick = skills.pivot_kick.PivotKick()
        kick.target = constants.Field.OurGoalSegment
        self.add_subbehavior(kick, 'kick', required=True)
    def on_exit_kicking(self):
        self.remove_subbehavior('kick')
