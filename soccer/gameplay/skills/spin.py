import robocup
import single_robot_composite_behavior
import skills.move
import behavior
import constants
import math
import time


## A simple behavior to make a robot move to a given point and face a given direction
# note: probably not overly useful in real plays, but is useful for testing purposes
class Spin(single_robot_composite_behavior.SingleRobotCompositeBehavior):

    def __init__(self, turnSpeed = math.pi / 2.0, period=5, pos = None):
        super().__init__(continuous=False)

        self.command_angles = list()
        self.real_angles = list()

        self.pos = pos
        self.period = period
        self.starttime = time.clock()
        self.turnSpeed = turnSpeed
        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')


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
        return self.elapsed_time()*self.turnSpeed

    @angle.setter
    def angle(self, value):
        self._angle = robocup.fix_angle_radians(value)%(math.pi / 2.0)

    def elapsed_time(self):
        return time.clock()-self.starttime

    def calculate_turn_speed(self):
        time = (self.elapsed_time()%self.period)/self.period*2
        if time<1:
            command = -self.turnSpeed + 2*time*self.turnSpeed;
            self.command_angles.append(command)
            #self.real_angles.append()
            return -self.turnSpeed + 2*time*self.turnSpeed
        else:
            return self.turnSpeed - (time-1)*self.turnSpeed*2
            self.command_angles = list()
            self.real_angles = list()

    def calculate_face_target(self):
        return self.pos + robocup.Point(math.cos(self.angle), math.sin(self.angle))

    def execute_running(self):
        if self.robot:
            self.robot.set_angle_vel(self.calculate_turn_speed())
