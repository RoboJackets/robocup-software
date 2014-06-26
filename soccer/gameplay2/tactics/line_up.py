import composite_behavior
import behavior
import skills.move
import robocup
import constants


class LineUp(composite_behavior.CompositeBehavior):

    def __init__(self, line=None):
        super().__init__(continuous=False)

        self._line = line

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')
        self.add_transition(behavior.Behavior.State.running,
            behavior.Behavior.State.completed,
            lambda: self.all_subbehaviors_completed(),
            'all robots reach target positions')
        self.add_transition(behavior.Behavior.State.completed,
            behavior.Behavior.State.running,
            lambda: not self.all_subbehaviors_completed(),
            'robots arent lined up')

        if self._line is None:
            self._line = robocup.Segment(
                robocup.Point(-constants.Field.Width/2 + constants.Robot.Radius, constants.Robot.Radius),
                robocup.Point(-constants.Field.Width/2 + constants.Robot.Radius, (constants.Robot.Radius * 2 + 0.1)*6))

        self.diff = (self._line.pt[1] - self._line.pt[0]).normalized() * ( self._line.length() / 6.0)

        # add subbehaviors for all robots, instructing them to line up
        for i in range(6):
            pt = self._line.pt[0] + ( self.diff * float(i)  )
            self.add_subbehavior(skills.move.Move(pt),
                name="robot" + str(i),
                required=False,
                priority=6 - i)

    def all_subbehaviors_completed(self):
        return all([b.behavior_state == behavior.Behavior.State.completed or b.robot == None for b in self.all_subbehaviors()])

    def execute_running(self):
        for i in range(6):
            pt = self._line.pt[0] + ( self.diff * float(i) )
            self.subbehavior_with_name("robot" + str(i)).pos = pt

    @property
    def line(self):
        return self._line
    @line.setter
    def line(self, value):
        self._line = value
        self.diff = (self._line.pt[1] - self._line.pt[0]).normalized() * ( self._line.length() / 6.0)