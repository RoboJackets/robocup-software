import single_robot_behavior
import behavior


# wraps up OurRobot.move() into a Skill so we can use it in the play system more easily
class Move(single_robot_behavior.SingleRobotBehavior):

    def __init__(self, pos=None):
        super().__init__(continuous=False)

        self.threshold = 0.05
        self.pos = pos

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True, 'immediately')

        self.add_transition(behavior.Behavior.State.running,
            behavior.Behavior.State.completed,
            lambda: self.pos != None and (self.robot.pos - self.pos).mag() < self.threshold,
            'target pos reached')
        self.add_transition(behavior.Behavior.State.completed,
            behavior.Behavior.State.running,
            lambda:  self.pos != None and (self.robot.pos - self.pos).mag() > self.threshold,
            'away from target')


    def execute_running(self):
        if self.pos != None:
            self.robot.move_to(self.pos)


    # the position to move to
    @property
    def pos(self):
        return self._pos
    @pos.setter
    def pos(self, value):
        self._pos = value


    # how close (in meters) the robot has to be to the target position for it be complete
    @property
    def threshold(self):
        return self._threshold
    @threshold.setter
    def threshold(self, value):
        self._threshold = value


    def role_requirements(self):
        reqs = super().role_requirements()
        reqs.destination_shape = self.pos
        return reqs
