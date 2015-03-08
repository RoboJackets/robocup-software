import play
import behavior
import tactics.spin
import robocup
import constants


class TestPivot(play.Play):

    def __init__(self):
        super().__init__(continuous=False)

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')
        self.add_transition(behavior.Behavior.State.running,
            behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name("Spin").state == behavior.Behavior.State.completed,
            'all robots reach target positions')
        self.add_transition(behavior.Behavior.State.completed,
            behavior.Behavior.State.running,
            lambda: self.subbehavior_with_name("Spin").state == behavior.Behavior.State.running,
            'robots arent lined up')

        x = (constants.Field.Width/2 - constants.Robot.Radius*2)
        y_start = 0.2
        line = robocup.Segment(
                robocup.Point(x, constants.Robot.Radius + y_start),
                robocup.Point(x, (constants.Robot.Radius * 2 + 0.1)*6 + y_start))
        l = tactics.spin.Spin(line)
        self.add_subbehavior(l, name="Spin", required=True)
