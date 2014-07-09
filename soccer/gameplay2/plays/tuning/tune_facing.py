import play
import robocup
import constants
import behavior
import enum
import math
import time


# This play rotates the bot 90 degrees, pauses, and repeats
# It's useful for tuning the angle PID controller
class TuneFacing(play.Play):

    PauseDuration = 2
    AngleIncrement = 90
    MaxAngleSpeed = None


    class State(enum.Enum):
        pause = 0
        turning = 1


    def __init__(self):
        super().__init__(continuous=True)

        for state in TuneFacing.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            TuneFacing.State.pause,
            lambda: True,
            'immediately')

        self.add_transition(TuneFacing.State.pause,
            TuneFacing.State.turning,
            lambda: time.time() - self.last_rotate_time > TuneFacing.PauseDuration,
            'pause over')

        self.add_transition(TuneFacing.State.turning,
            TuneFacing.State.pause,
            lambda: self.robot.angle - self.face_angle < 5 and self.robot.angleVel < 5,
            'facing and settled')

        self.last_rotate_time = time.time()
        self.face_angle = 0
        self.point = robocup.Point(0, constants.Field.Length / 4.0)



    @property
    def last_rotate_time(self):
        return self._last_rotate_time
    @last_rotate_time.setter
    def last_rotate_time(self, value):
        self._last_rotate_time = value


    # angle in degrees
    @property
    def face_angle(self):
        return self._face_angle
    @face_angle.setter
    def face_angle(self, value):
        self._face_angle = value


    def calculate_face_target(self):
        angle_rad = self.face_angle * constants.DegreesToRadians
        offset = robocup.Point(math.cos(angle_rad), math.sin(angle_rad))


    # where the robot sits on the field as it rotates
    # Default: center of our half of the field
    @property
    def point(self):
        return self._point
    @point.setter
    def point(self, value):
        self._point = value


    def execute_turning(self):
        self.face(self.calculate_face_target())

    def on_exit_turning(self):
        self.face_angle += AngleIncrement
        self.last_rotate_time = time.time()
    

    def execute_running(self):
        self.robot.move_to(self.point)
