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
        self.times = list()

        self.pos = pos
        self.period = period
        self.starttime = time.clock()
        self.turnSpeed = turnSpeed

        self.turnClockwise = 0
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
        time = (self.elapsed_time()%self.period)
        time_sec = time
        time = time/self.period*2
        if time<1:
            if self.turnClockwise==0:
                self.command_angles = list()
                self.real_angles = list()
                self.times = list()
                self.turnClockwise = 1
                self.iteration = 0
                self.zeroTime = -1
                self.realZeroTime = -1
            command = -self.turnSpeed + 2*time*self.turnSpeed;

            self.times.append(time_sec)
            self.command_angles.append(command)
            self.real_angles.append(self.robot.angle_vel)
            if command >=0 and self.zeroTime==-1:
                print ("fake")
                right = command
                left = -self.command_angles[self.iteration-1]
                print (left)
                print (right)
                total = right + left
                right = right/total
                left = left/total
                self.zeroTime = self.times[self.iteration]*left + self.times[self.iteration-1]*right
                print (self.zeroTime)

            if  time>0.3 and self.robot.angle_vel >= 0 and self.realZeroTime == -1:
                print ("real")
                right = self.real_angles[self.iteration]
                left = -self.real_angles[self.iteration-1]
                print (left)
                print (right)
                total = right + left
                right = right/total
                left = left/total
                self.realZeroTime = self.times[self.iteration]*left + self.times[self.iteration-1]*right
                print (self.realZeroTime)


            
            self.iteration += 1
            return command
        else:
            if self.turnClockwise:
                self.turnClockwise = 0
            return self.turnSpeed - (time-1)*self.turnSpeed*2

    def calculate_face_target(self):
        return self.pos + robocup.Point(math.cos(self.angle), math.sin(self.angle))

    def execute_running(self):
        if self.robot:
            self.robot.set_angle_vel(self.calculate_turn_speed())

#            if self.turnClockwise:
