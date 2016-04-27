import single_robot_composite_behavior
import behavior
import robocup
import role_assignment
import main
import constants
import time
import enum
import skills.aim
import skills.capture

## Behavior that moves the ball to a specified location
class Dribble(single_robot_composite_behavior.SingleRobotCompositeBehavior):

    class State(enum.Enum):
        capture=1
        aim=2
        drive=3

    def __init__(self, pos=None, vel=0):
        super().__init__(continuous=False)

        for state in Dribble.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.threshold = 0.01 #default value matches the required accuracy for a placement command
        self.pos = pos
        self.vel = vel

        self.add_transition(behavior.Behavior.State.start,
                            Dribble.State.capture, lambda: True,
                            'immediately')

        self.add_transition(
            Dribble.State.capture, Dribble.State.aim,
            lambda: self.subbehavior_with_name('capture').state == behavior.Behavior.State.completed,
            'done capturing')

        #Put the ball between the robot and the target
        self.add_transition(
            Dribble.State.aim, Dribble.State.drive,
            lambda: self.subbehavior_with_name('aim').state == skills.aim.Aim.State.aimed,
            'done aiming')

        self.add_transition(
            Dribble.State.aim, Dribble.State.capture,
            lambda: self.subbehavior_with_name("aim").state == behavior.Behavior.State.failed,
            'fumbled')

        self.add_transition(
            Dribble.State.drive, Dribble.State.capture, lambda: self.fumbled() or self.robot.vel.mag()>1, 'fumbled')

        self.add_transition(
            Dribble.State.drive, behavior.Behavior.State.completed,
            lambda: (main.ball().pos - self.pos).mag() < self.threshold,
            'finished driving')

        self.last_ball_time = 0


    def fumbled(self):
        return not self.robot.has_ball() and time.time(
        ) - self.last_ball_time > 0.1

    ## the position to move to (a robocup.Point object)
    @property
    def pos(self):
        return self._pos

    @property
    def vel(self):
        return self._vel

    @pos.setter
    def pos(self, value):
        self._pos = value

    @vel.setter
    def vel(self, value):
        self._vel=value;

    ## how close (in meters) the robot has to be to the target position for it be complete
    @property
    def threshold(self):
        return self._threshold

    @threshold.setter
    def threshold(self, value):
        self._threshold = value

    def on_enter_capture(self):
        capture=skills.capture.Capture()
        self.add_subbehavior(capture, 'capture', required=True, priority=100)

    def on_exit_capture(self):
        self.remove_subbehavior('capture')


    def on_enter_aim(self):
        aim=skills.aim.Aim()
        aim.target_point = self.pos
        self.add_subbehavior(aim,'aim', required=True, priority=100)

        if self.robot.has_ball():
            self.last_ball_time = time.time()

    def on_exit_aim(self):
        self.remove_subbehavior('aim')


    def on_enter_drive(self):
        print("in drive")
        self.robot.set_dribble_speed(int(constants.Robot.Dribbler.MaxPower))
    
    def execute_drive(self):
        print("executing drive")
        self.robot.face(self.pos)
        self.robot.move_to_direct_end_vel(self.pos,self.vel)
        if self.robot.has_ball():
            self.last_ball_time = time.time()

    #Robot closest to the ball
    def role_requirements(self):
        reqs = super().role_requirements()
        if isinstance(reqs, role_assignment.RoleRequirements):
            reqs.destination_shape = main.ball().pos

        return reqs
