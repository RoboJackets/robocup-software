import composite_behavior
import behavior
import skills.move
import robocup
import constants
from single_robot_behavior import SingleRobotBehavior


class SetFormation(composite_behavior.CompositeBehavior):

    DEBOUNCE_MAX = 3
    # y_start = 1.2  # sometimes we have issues if we're right in the corner, so we move it up a bit
    DefaultFormation = [robocup.Point(-2, 4.5), robocup.Point(0, 4.5), robocup.Point(2, 4.5),
        robocup.Point(-1, 3), robocup.Point(1, 3), robocup.Point(0, 1)] #random default values
    
    def __init__(self, formation=None):
        super().__init__(continuous=False)

        #formation is a list of robocup.Point objects corresponding to desired positions
        self.formation = formation if formation is not None else SetFormation.DefaultFormation
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
                            'robots havent reached target positions')

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
        if self.debounce_count < SetFormation.DEBOUNCE_MAX:
            self.debounce_count += 1
            return False
        else:
            return True

    def execute_running(self):
        for i in range(6):
            # pt = self._line.get_pt(0) + (self.diff * float(i))
            pt = self.formation[i]
            behavior = self.subbehavior_with_name("robot" + str(i))
            behavior.pos = pt

    @property
    def formation(self):
        return self._formation

    @formation.setter
    def formation(self, value):
        self._formation = value

        # add subbehaviors for all robots, instructing them to line up
        self.remove_all_subbehaviors()
        for i in range(6):
            pt = self._formation[i]
            self.add_subbehavior(
                skills.move.Move(pt),
                name="robot" + str(i),
                required=False,
                priority=6-i)
