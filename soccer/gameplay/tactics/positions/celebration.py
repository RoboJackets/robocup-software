import behavior
import robocup
import constants
import single_robot_composite_behavior
import main
import enum
import skills
import random
import time


class Celebration(
        single_robot_composite_behavior.SingleRobotCompositeBehavior):
    MaxSpinAngle = 360
    SpinPerTick = 1

    class State(enum.Enum):
        run_around = 0

        spin = 1

    def __init__(self):
        super().__init__(continuous=True)

        for s in Celebration.State:
            self.add_state(s, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            Celebration.State.run_around, lambda: True,
                            'immediately')

        self.add_transition(Celebration.State.run_around,
                            Celebration.State.spin, lambda: self.spin_time,
                            'time to yeet')

        self.add_transition(
            Celebration.State.spin,
            Celebration.State.run_around, lambda: self.im_dizzy(),
            'time to go running')

        self.spin_angle = 0
        self.spin_time = False
        r = constants.Robot.Radius
        self.corners = [
            robocup.Point(-constants.Field.Width / 2 + r, r),
            robocup.Point(constants.Field.Width / 2 - r, r), robocup.Point(
                constants.Field.Width / 2 - r, constants.Field.Length - r),
            robocup.Point(-constants.Field.Width / 2 + r,
                          constants.Field.Length - r),
            robocup.Point(0, constants.Field.Length / 2)
        ]

        self.current_corner = 0
        self.start_time = time.time()

    def on_enter_run_around(self):
        self.current_corner = random.randint(0, 4)
        self.robot.move_to_direct(self.corners[0])

    def execute_run_around(self):
        if (self.robot.pos - self.corners[self.current_corner]).mag() <= .05:
            if (self.current_corner == 4):
                self.spin_time = True
                self.current_corner = random.randint(0, 3)
            else:
                self.current_corner = random.randint(0, 4)

        if (self.current_corner < 5):
            self.robot.move_to_direct(self.corners[self.current_corner])

    def on_enter_spint(self):
        self.start_time = time.time()

    def execute_spin(self):
        angle = self.robot.angle
        facing_point = robocup.Point.direction(angle) + self.robot.pos
        facing_point.rotate(self.robot.pos, Celebration.SpinPerTick)
        self.spin_angle += Celebration.SpinPerTick
        self.robot.face(facing_point)

    def im_dizzy(self):
        return time.time() - self.start_time > 8 and time.time(
        ) - self.start_time < 20
