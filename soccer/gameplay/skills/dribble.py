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
        capture = 1
        aim = 2
        drive = 3

    def __init__(self, pos=None):
        super().__init__(continuous=False)

        for state in Dribble.State:
            self.add_state(state, behavior.Behavior.State.running)

        self._threshold = 0.1  #default value matches the required accuracy for a placement command
        self._pos = pos

        self._dribble_speed = constants.Robot.Dribbler.MaxPower

        self.add_transition(behavior.Behavior.State.start,
                            Dribble.State.capture, lambda: True, 'immediately')

        self.add_transition(
            Dribble.State.capture, Dribble.State.aim,
            lambda: self.subbehavior_with_name('capture').state == behavior.Behavior.State.completed,
            'done capturing')

        self.add_transition(
            Dribble.State.capture, behavior.Behavior.State.completed,
            lambda: (main.ball().pos - self.pos).mag() < self._threshold and main.ball().vel.mag() < .1,
            'ball is already in target')

        #Put the ball between the robot and the target
        self.add_transition(Dribble.State.aim, Dribble.State.drive,
                            lambda: self.aimed() and self.robot.has_ball(),
                            'done aiming')

        self.add_transition(Dribble.State.aim, Dribble.State.capture,
                            lambda: self.fumbled(), 'fumbled')

        self.add_transition(Dribble.State.drive, Dribble.State.capture,
                            lambda: self.fumbled(), 'fumbled')

        self.add_transition(
            Dribble.State.drive, behavior.Behavior.State.completed,
            lambda: (main.ball().pos - self.pos).mag() < self._threshold - constants.Ball.Radius and not self.fumbled() and main.ball().vel.mag() < .1,
            'finished driving')

        self.last_ball_time = 0

    def fumbled(self):
        return not self.robot.has_ball() and time.time(
        ) - self.last_ball_time > 0.3

    ## the position to move to (a robocup.Point object)
    @property
    def pos(self):
        return self._pos

    @pos.setter
    def pos(self, value):
        self._pos = value

    def set_dribble_speed(self, value):
        self._dribble_speed = value

    ## how close (in meters) the robot has to be to the target position for it be complete
    @property
    def threshold(self):
        return self._threshold

    @threshold.setter
    def threshold(self, value):
        self._threshold = value

    def aimed(self):
        angle = self.robot.angle - (self.pos - self.robot.pos).angle()
        return angle < .1 and angle > -.1

    def on_enter_capture(self):
        self.robot.unkick()
        capture = skills.capture.Capture()
        self.add_subbehavior(capture, 'capture', required=True, priority=100)

    def on_exit_capture(self):
        self.remove_subbehavior('capture')

    def execute_aim(self):
        self.robot.set_max_angle_speed(2)
        self.robot.pivot(self.pos)
        self.robot.set_dribble_speed(self._dribble_speed)
        if self.robot.has_ball():
            self.last_ball_time = time.time()

    def execute_drive(self):
        self.robot.set_dribble_speed(self._dribble_speed)
        self.robot.face(self.pos)

        #self.robot.set_max_speed(1.0)
        self.robot.set_max_speed(.25)

        #offset by the size of the robot so the ball is on the target position when it stops
        self.robot.disable_avoid_ball()
        self.robot.move_to(self.pos - (self.pos - self.robot.pos).normalized(
            constants.Robot.Radius+0.08))
        if self.robot.has_ball():
            self.last_ball_time = time.time()

    #Robot closest to the ball
    def role_requirements(self):
        reqs = super().role_requirements()
        if isinstance(reqs, role_assignment.RoleRequirements):
            reqs.destination_shape = main.ball().pos

        return reqs
