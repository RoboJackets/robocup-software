import play
import single_robot_behavior
import robocup
import constants
import behavior
import enum
import math
import time


class Pivoter(single_robot_behavior.SingleRobotBehavior):
    def __init__(self):
        super().__init__(continuous=True)

        self.angle = 0
        point = robocup.Point(0, constants.Field.Length / 4.0)
        self.face_target = point + robocup.Point(
            math.cos(self.angle), math.sin(self.angle))

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

    # # where the robot sits on the field as it rotates
    # # Default: center of our half of the field
    # @property
    # def point(self):
    #     return self._point
    # @point.setter
    # def point(self, value):
    #     self._point = value

    def execute_running(self):
        self.robot.set_max_angle_speed(5 * constants.DegreesToRadians)
        self.robot.pivot(self.face_target)


# This play rotates the bot 90 degrees, pauses, and repeats
# It's useful for tuning the angle PID controller
class TunePivoting(play.Play):
    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        rotater = Pivoter()
        self.add_subbehavior(rotater, 'rotater', required=False)

    def execute_running(self):
        rotater = self.subbehavior_with_name('rotater')
        if rotater.robot != None:
            rotater.robot.set_dribble_speed(int(
                constants.Robot.Dribbler.MaxPower / 3))
