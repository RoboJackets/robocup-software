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
import skills.move


## Behavior that moves the ball to a specified location
class Dribble(single_robot_composite_behavior.SingleRobotCompositeBehavior):
    ENTER_DRIVE_MAX_SPEED = .5
    DRIVE_MAX_SPEED = .3

    class State(enum.Enum):
        setup = 1
        capture = 2
        aim = 3
        drive = 4

    def __init__(self, pos=None):
        super().__init__(continuous=False)

        for state in Dribble.State:
            self.add_state(state, behavior.Behavior.State.running)

        self._threshold = 0.1  #default value matches the required accuracy for a placement command
        self._pos = pos

        # Tested with dribbler speed of 70
        self._dribble_speed = constants.Robot.Dribbler.StandardPower

        self.add_transition(behavior.Behavior.State.start, Dribble.State.setup,
                            lambda: True, 'immediately')

        self.add_transition(
            Dribble.State.setup, Dribble.State.capture,
            lambda: (self.subbehavior_with_name('move').state == behavior.Behavior.State.completed),
            'moved to behind the ball')

        self.add_transition(
            Dribble.State.capture, Dribble.State.aim,
            lambda: (self.subbehavior_with_name('capture').state == behavior.Behavior.State.completed),
            'done capturing')

        self.add_transition(
            Dribble.State.capture, behavior.Behavior.State.completed,
            lambda: ((main.ball().pos - self.pos).mag() < self._threshold and main.ball().vel.mag() < .1),
            'ball is already in target')

        #Put the ball between the robot and the target
        self.add_transition(Dribble.State.aim, Dribble.State.drive,
                            lambda: self.aimed() and self.robot.has_ball(),
                            'done aiming')

        self.add_transition(Dribble.State.aim, Dribble.State.setup,
                            lambda: self.fumbled(), 'fumbled')

        self.add_transition(Dribble.State.drive, Dribble.State.setup,
                            lambda: self.fumbled(), 'fumbled')

        self.add_transition(
            Dribble.State.capture, Dribble.State.setup,
            lambda: (self.robot.pos - main.ball().pos).mag() > 1, 'fumbled')

        self.add_transition(
            Dribble.State.drive, behavior.Behavior.State.completed,
            lambda: (self.subbehavior_with_name('dmove').state == behavior.Behavior.State.completed),
            'finished driving')

        self.last_ball_time = 0

    def fumbled(self):
        return not self.robot.has_ball() and time.time(
        ) - self.last_ball_time > 1.2

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

    def on_enter_setup(self):
        self.move_vector = main.ball().pos - self._pos
        self.move_point = main.ball().pos + self.move_vector.normalized() * .2
        move = skills.move.Move(self.move_point)
        self.add_subbehavior(move, 'move', required=True, priority=100)

    def execute_setup(self):
        main.system_state().draw_circle(self.move_point, 0.1,
                                        constants.Colors.Blue, "move setup")

    def on_exit_setup(self):
        self.remove_all_subbehaviors()

    def on_enter_capture(self):
        self.robot.unkick()
        capture = skills.capture.Capture()
        self.add_subbehavior(capture, 'capture', required=True, priority=100)

    def on_exit_capture(self):
        self.remove_all_subbehaviors()

    def execute_aim(self):
        self.robot.set_max_angle_speed(2)
        self.robot.pivot(self.pos)
        self.robot.set_dribble_speed(self._dribble_speed)
        if self.robot.has_ball():
            self.last_ball_time = time.time()

    def on_enter_drive(self):
        move = skills.move_direct.MoveDirect(self.pos - (
            self.pos - self.robot.pos).normalized(constants.Robot.Radius))

        self.add_subbehavior(move, 'dmove', required=True, priority=100)
        self.robot.set_max_speed(Dribble.ENTER_DRIVE_MAX_SPEED)

    def execute_drive(self):
        self.robot.set_dribble_speed(self._dribble_speed)
        self.robot.face(self.pos)
        self.robot.set_max_speed(Dribble.DRIVE_MAX_SPEED)

        #offset by the size of the robot so the ball is on the target position when it stops
        self.robot.disable_avoid_ball()

        if self.robot.has_ball():
            self.last_ball_time = time.time()

    def on_exit_drive(self):
        self.remove_all_subbehaviors()
        print("exit dribbler")

    #Robot closest to the ball
    def role_requirements(self):
        reqs = super().role_requirements()
        if isinstance(reqs, role_assignment.RoleRequirements):
            reqs.destination_shape = main.ball().pos

        return reqs
