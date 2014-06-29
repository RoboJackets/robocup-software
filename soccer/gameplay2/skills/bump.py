import single_robot_behavior
import behavior
import constants
import main
import enum


# pushes the ball by bumping into it
class Bump(single_robot_behavior.SingleRobotBehavior):

    class State(enum.Enum):
        lineup = 1
        charge = 2


    def __init__(self):
        super().__init__(continuous=False)

        self.target = robocup.Point(0, constants.Field.Length)

        self.add_state(Bump.State.lineup, behavior.Behavior.State.running)
        self.add_state(Bump.State.charge, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            Bump.State.lineup,
            lambda: True,
            'immediately')

        self.add_transition(Bump.State.lineup,
            Bump.State.charge,
            lambda: False,
            '')

        self.add_transition(Bump.State.charge,
            behavior.Behavior.State.completed,
            lambda: False,
            '')


    def execute_charge(self):
        main.system_state().draw_line(robocup.Line(self.robot.pos, self.target),
            constants.Colors.White,
            "bump")
        main.system_state().draw_line(robocup.Line(main.ball().pos, self.target),
            constants.Colors.White,
            "bump")

        ball2target = (target - main.ball().pos).normalized()
        drive_dir = (main.ball().pos - ball2target * constants.Robot.Radius) - self.robot.pos

        speed = self.robot.vel.mag() + AccelBias



    # the Point we're trying to bump the ball towards
    @property
    def target(self):
        return self._target
    @target.setter
    def target(self, value):
        self._target = value
    
