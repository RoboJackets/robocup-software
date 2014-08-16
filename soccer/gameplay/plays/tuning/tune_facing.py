import play
import single_robot_behavior
import robocup
import constants
import behavior
import enum
import math
import time


class Facer(single_robot_behavior.SingleRobotBehavior):
    def __init__(self):
        super().__init__(continuous=True)

        self.angle = math.pi / 2.0
        self.point = robocup.Point(0, constants.Field.Length / 4.0)
        self.face_target = self.point + robocup.Point(math.cos(self.angle), math.sin(self.angle))

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')


    # where the robot sits on the field as it rotates
    # Default: center of our half of the field
    @property
    def point(self):
        return self._point
    @point.setter
    def point(self, value):
        self._point = value

    def execute_running(self):
        self.robot.move_to(self.point)
        self.robot.set_max_angle_speed(10 * constants.DegreesToRadians)
        self.robot.face(self.face_target)



# This play rotates the bot 90 degrees, pauses, and repeats
# It's useful for tuning the angle PID controller
class TuneFacing(play.Play):
    def __init__(self):
        super().__init__(continuous=True)

        rotater = Facer()
        self.add_subbehavior(rotater, 'rotater', required=False)
   