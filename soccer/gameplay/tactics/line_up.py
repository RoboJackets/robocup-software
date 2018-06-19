import composite_behavior
import behavior
import skills.move
import robocup
import constants
from single_robot_behavior import SingleRobotBehavior


class LineUp(composite_behavior.CompositeBehavior):

    DEBOUNCE_MAX = 3
    y_start = 1.2  # sometimes we have issues if we're right in the corner, so we move it up a bit
    DefaultLine = robocup.Segment(
        robocup.Point(-constants.Field.Width / 2 + constants.Robot.Radius,
                      constants.Robot.Radius + y_start),
        robocup.Point(-constants.Field.Width / 2 + constants.Robot.Radius,
                      (constants.Robot.Radius * 2 + 0.1 + y_start) * 6))

    def __init__(self, line=None):
        super().__init__(continuous=False)

        self.line = line if line is not None else LineUp.DefaultLine
        self.debounce_count = 0

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

    def restart(self):
        super().restart()
        self.debounce_count = 0

    # override superclass implementation of all_subbehaviors_completed() to
    # count unassigned subbehaviors as "done running"
    def all_subbehaviors_completed(self):
        for bhvr in self.all_subbehaviors():
            if not bhvr.is_done_running() and (
                    not isinstance(bhvr, SingleRobotBehavior) or
                    bhvr.robot is not None):
                self.debounce_count = 0
                return False
        # Give us a few cycles to change our mind
        if self.debounce_count < LineUp.DEBOUNCE_MAX:
            self.debounce_count += 1
            return False
        else:
            return True

    def execute_running(self):
        for i in range(6):
            pt = self._line.get_pt(0) + (self.diff * float(i))
            behavior = self.subbehavior_with_name("robot" + str(i))
            behavior.pos = pt

    @property
    def line(self):
        return self._line

    @line.setter
    def line(self, value):
        self._line = value
        self.diff = (
            self._line.get_pt(1) - self._line.get_pt(0)).normalized() * (
                self._line.length() / 5.0)

        # add subbehaviors for all robots, instructing them to line up
        self.remove_all_subbehaviors()
        for i in range(6):
            pt = self._line.get_pt(0) + (self.diff * float(i))
            self.add_subbehavior(
                skills.move.Move(pt),
                name="robot" + str(i),
                required=False,
                priority=6 - i)
