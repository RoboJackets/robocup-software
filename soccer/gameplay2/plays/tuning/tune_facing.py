import play
import single_robot_behavior
import robocup
import constants
import behavior
import enum
import math
import time




# class Rotater(single_robot_behavior.SingleRobotBehavior):

#     PauseDuration = 2
#     AngleIncrement = 90
#     MaxAngleSpeed = None


#     class State(enum.Enum):
#         pause = 0
#         turning = 1


#     def __init__(self):
#         super().__init__(continuous=True)

#         for state in Rotater.State:
#             self.add_state(state, behavior.Behavior.State.running)

#         self.add_transition(behavior.Behavior.State.start,
#             Rotater.State.pause,
#             lambda: True,
#             'immediately')

#         self.add_transition(Rotater.State.pause,
#             Rotater.State.turning,
#             lambda: time.time() - self.last_rotate_time > Rotater.PauseDuration,
#             'pause over')

#         self.add_transition(Rotater.State.turning,
#             Rotater.State.pause,
#             lambda: self.robot.angle - self.face_angle < 5 and self.robot.angle_vel < 5,
#             'facing and settled')

#         self.last_rotate_time = time.time()
#         self.face_angle = 0
#         self.point = robocup.Point(0, constants.Field.Length / 4.0)



#     @property
#     def last_rotate_time(self):
#         return self._last_rotate_time
#     @last_rotate_time.setter
#     def last_rotate_time(self, value):
#         self._last_rotate_time = value


#     # angle in degrees
#     @property
#     def face_angle(self):
#         return self._face_angle
#     @face_angle.setter
#     def face_angle(self, value):
#         self._face_angle = value


#     def calculate_face_target(self):
#         angle_rad = self.face_angle * constants.DegreesToRadians
#         offset = robocup.Point(math.cos(angle_rad), math.sin(angle_rad))
#         return self.point + offset


#     # where the robot sits on the field as it rotates
#     # Default: center of our half of the field
#     @property
#     def point(self):
#         return self._point
#     @point.setter
#     def point(self, value):
#         self._point = value


#     def execute_turning(self):
#         self.robot.face(self.calculate_face_target())

#     def on_exit_turning(self):
#         self.face_angle += Rotater.AngleIncrement
#         self.last_rotate_time = time.time()
    

#     def execute_running(self):
#         self.robot.move_to(self.point)




class Facer(single_robot_behavior.SingleRobotBehavior):
    def __init__(self):
        super().__init__(continuous=True)

        self.angle = -90
        self.point = robocup.Point(0, constants.Field.Length / 4.0)
        self.face_target = self.point + robocup.Point(math.cos(self.angle * constants.DegreesToRadians), math.sin(self.angle * constants.DegreesToRadians))

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
        self.robot.set_max_angle_speed(10)
        self.robot.face(self.face_target)



# This play rotates the bot 90 degrees, pauses, and repeats
# It's useful for tuning the angle PID controller
class TuneFacing(play.Play):
    def __init__(self):
        super().__init__(continuous=True)

        rotater = Facer()
        self.add_subbehavior(rotater, 'rotater', required=False)
   