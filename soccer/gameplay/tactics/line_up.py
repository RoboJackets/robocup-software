import composite_behavior
import behavior
import skills.move
import robocup
import constants


class LineUp(composite_behavior.CompositeBehavior):

    y_start = 1.0  # sometimes we have issues if we're right in the corner, so we move it up a bit
    DefaultLine = robocup.Segment(
        robocup.Point(-constants.Field.Width / 2 + constants.Robot.Radius,
                      constants.Robot.Radius + y_start),
        robocup.Point(-constants.Field.Width / 2 + constants.Robot.Radius,
                      (constants.Robot.Radius * 2 + 0.1 + y_start) * 6))

    def __init__(self, line=None):
        super().__init__(continuous=False)

        self.line = line if line != None else LineUp.DefaultLine

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')
        self.add_transition(behavior.Behavior.State.running,
                            behavior.Behavior.State.completed,
                            lambda: self.all_subbehaviors_completed(),
                            'all robots reach target positions')
        self.add_transition(behavior.Behavior.State.completed,
                            behavior.Behavior.State.running,
                            lambda: not self.all_subbehaviors_completed(),
                            'robots arent lined up')

    def execute_running(self):
        for i in range(6):
            pt = self._line.get_pt(0) + (self.diff * float(i))
            self.subbehavior_with_name("robot" + str(i)).pos = pt

    @property
    def line(self):
        return self._line

    @line.setter
    def line(self, value):
        self._line = value
        self.diff = (
            self._line.get_pt(1) - self._line.get_pt(0)).normalized() * (
                self._line.length() / 6.0)

        # add subbehaviors for all robots, instructing them to line up
        self.remove_all_subbehaviors()
        for i in range(6):
            pt = self._line.get_pt(0) + (self.diff * float(i))
            self.add_subbehavior(
                skills.move.Move(pt),
                name="robot" + str(i),
                required=False,
                priority=6 - i)
