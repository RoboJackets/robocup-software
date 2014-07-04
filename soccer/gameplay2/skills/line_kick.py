import single_robot_behavior
import behavior
import constants
import robocup
import enum


# lines up with the ball and the target, then drives up and kicks
# this differs from PivotKick which gets the ball first, then aims
class LineKick(single_robot_behavior.SingleRobotBehavior):

    class State(enum.Enum):
        setup = 1
        charge = 2


    def __init__(self):
        super().__init__(continuous=False)
        self.target_point = constants.Field.TheirGoalSegment.center()

        for state in LineKick.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            LineKick.State.setup,
            lambda: True,
            'immediately')



    # where we're trying to kick
    # Default: the center of their goal
    @property
    def target_point(self):
        return self._target_point
    @target_point.setter
    def target_point(self, value):
        self._target_point = value


    def recalculate(self):
        self._target_line = robocup.Line(main.ball().pos, self.target_point)

