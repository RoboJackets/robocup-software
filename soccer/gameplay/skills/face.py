import robocup
import single_robot_composite_behavior
import skills.move
import behavior
import constants
import math


## A simple behavior to make a robot move to a given point and face a given direction
# note: probably not overly useful in real plays, but is useful for testing purposes
class Face(single_robot_composite_behavior.SingleRobotCompositeBehavior):

    def __init__(self, pos = robocup.Point(0, constants.Field.Length / 4.0), angle = math.pi / 2.0):
        super().__init__(continuous=False)

        self.angle = angle
        self.pos = pos

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')
        self.add_transition(behavior.Behavior.State.running,
            behavior.Behavior.State.completed,
            lambda: self.is_at_target_angle() and self.subbehavior_with_name('move').state == behavior.Behavior.State.completed,
            'at target pos and angle')
        self.add_transition(behavior.Behavior.State.completed,
            behavior.Behavior.State.running,
            lambda: not self.is_at_target_angle() or self.subbehavior_with_name('move').state != behavior.Behavior.State.completed,
            'not at target pos and angle')

        m = skills.move.Move(self.pos)
        self.add_subbehavior(m, 'move')


    ## The position to move to
    @property
    def pos(self):
        return self._pos
    @pos.setter
    def pos(self, value):
        self._pos = value
        if self.has_subbehavior_with_name('move'):
            self.subbehavior_with_name('move').pos = self.pos


    ## The angle (in radians) to face
    @property
    def angle(self):
        return self._angle
    @angle.setter
    def angle(self, value):
        self._angle = robocup.fix_angle_radians(value)


    def is_at_target_angle(self):
        if self.robot != None:
            diff = abs(robocup.fix_angle_radians(self.robot.angle - self.angle))
            return diff < (math.pi / 64.0)
        else:
            return False


    def calculate_face_target(self):
        return self.pos + robocup.Point(math.cos(self.angle), math.sin(self.angle))


    def execute_running(self):
        self.robot.face(self.calculate_face_target())
