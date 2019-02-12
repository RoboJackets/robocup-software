import single_robot_behavior
import behavior
import constants
import main
import enum
import robocup
import math


## pushes the ball by bumping into it
class Bump(single_robot_behavior.SingleRobotBehavior):

    # tuneable constants
    DriveAroundDist = 0.35  # how far away from the ball we should stay in the lineup state
    LineupBallAvoidRadius = 0.043  # ball avoid radius
    FacingThresh = 10 * constants.DegreesToRadians  # angle in radians that we must be less than away from the ball
    AccelBias = 0.1  # how much to add to our desired speed
    EscapeChargeThresh = 0.1  # if we're off by this much, we go back to the lineup state
    LineupToChargeThresh = 0.05  # how close we have to be to our target line to enter the charge state

    class State(enum.Enum):
        lineup = 1
        charge = 2

    def __init__(self):
        super().__init__(continuous=False)

        self.target = robocup.Point(0, constants.Field.Length)

        self.add_state(Bump.State.lineup, behavior.Behavior.State.running)
        self.add_state(Bump.State.charge, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start, Bump.State.lineup,
                            lambda: True, 'immediately')

        self.add_transition(
            Bump.State.lineup, Bump.State.charge,
            lambda: self.target_line().dist_to(self.robot.pos) <= Bump.LineupToChargeThresh and self.target_line().delta().dot(self.robot.pos - main.ball().pos) <= -constants.Robot.Radius and not self.facing_err_above_threshold(),
            'lined up')

        # FIXME: this condition was never setup in the C++ one...
        self.add_transition(
            Bump.State.charge, behavior.Behavior.State.completed,
            lambda: (main.ball().pos - self.robot.pos).mag() < (constants.Robot.Radius + constants.Ball.Radius + 0.03),
            'ball has been bumped')

        self.add_transition(
            Bump.State.charge, Bump.State.lineup,
            lambda: robocup.Line(self.robot.pos, self.target).dist_to(main.ball().pos) > Bump.EscapeChargeThresh,
            'bad ball placement')

    ## the Point we're trying to bump the ball towards
    @property
    def target(self):
        return self._target

    @target.setter
    def target(self, value):
        self._target = value

    def facing_err_above_threshold(self):
        dirVec = robocup.Point(
            math.cos(self.robot.angle), math.sin(self.robot.angle))
        facing_thresh = math.cos(Bump.FacingThresh)
        facing_err = dirVec.dot((self.target - main.ball().pos).normalized())
        # NOTE: yes, the comparator is backwards, that's the way it was in the c++ one...
        # TODO: remove the above note and clarify what's going on
        return facing_err < facing_thresh

    # line from ball to target
    def target_line(self):
        return robocup.Line(main.ball().pos, self.target)

    def execute_lineup(self):
        target_line = self.target_line()
        target_dir = target_line.delta().normalized()
        behind_line = robocup.Segment(
            main.ball().pos - target_dir *
            (Bump.DriveAroundDist + constants.Robot.Radius),
            main.ball().pos - target_dir * 5.0)
        if target_line.delta().dot(self.robot.pos - main.ball(
        ).pos) > -constants.Robot.Radius:
            # we're very close to or in front of the ball
            self.robot.set_avoid_ball_radius(Bump.LineupBallAvoidRadius)
            self.robot.move_to(main.ball().pos - target_dir * (
                Bump.DriveAroundDist + constants.Robot.Radius))
        else:
            self.robot.set_avoid_ball_radius(Bump.LineupBallAvoidRadius)
            self.robot.move_to(behind_line.nearest_point(self.robot.pos))
            main.system_state().draw_line(behind_line, constants.Colors.Black,
                                          "Bump")

        delta_facing = self.target - main.ball().pos
        self.robot.face(self.robot.pos + delta_facing)

    def execute_charge(self):
        main.system_state().draw_line(
            robocup.Line(self.robot.pos, self.target), constants.Colors.White,
            "bump")
        main.system_state().draw_line(
            robocup.Line(main.ball().pos, self.target), constants.Colors.White,
            "bump")

        ball2target = (self.target - main.ball().pos).normalized()
        drive_dir = (main.ball().pos - ball2target *
                     constants.Robot.Radius) - self.robot.pos

        # we want to drive toward the ball without using the path planner
        # we do this by setting the speed directly
        # AccelBias forces us to accelerate a bit more
        speed = self.robot.vel.mag() + Bump.AccelBias

        self.robot.set_world_vel(drive_dir.normalized() * speed)
        self.robot.face(main.ball().pos)
