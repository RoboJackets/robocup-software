import skills._kick
import behavior
import constants
import robocup
import enum


# lines up with the ball and the target, then drives up and kicks
# this differs from PivotKick which gets the ball first, then aims
class LineKick(skills._kick._Kick):

    # tuneable constants
    DriveAroundDist = 0.15
    ChargeThresh = 0.1
    EscapeChargeThresh = 0.1
    SetupBallAvoid = 0.08
    AccelBias = 0.2
    FacingThresh = 10   # angle in degrees
    MaxChargeSpeed = 1.5
    BallProjectTime = 0.4
    DoneStateThresh = 0.11
    LandOnTarget = False


    class State(enum.Enum):
        setup = 1
        charge = 2


    def __init__(self):
        super().__init__()
        self.target_point = constants.Field.TheirGoalSegment.center()

        for state in LineKick.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            LineKick.State.setup,
            lambda: True,
            'immediately')



    # where we're trying to kick
    # Default: the center of their goal
    @property
    def target_point(self):
        return self._target_point
    @target_point.setter
    def target_point(self, value):
        self._target_point = value


    def recalculate(self):
        self._target_line = robocup.Line(main.ball().pos, self.target_point)

        bot_dir = robocup.Point.direction(self.robot.angle * constants.DegreesToRadians)


    def execute_running(self):
        self.recalculate()


    def execute_setup(self):
        move_goal = main.ball().pos - self._target_line.delta().normalized() * (LineKick.DriveAroundDist + constants.Robot.Radius)

        left_field_edge = robocup.Segment(robocup.Point(-constants.Field.Width / 2.0, 0),
            robocup.Point(-constants.Field.Width / 2.0, constants.Field.Length))
        right_field_edge = robocup.Segment(robocup.Point(constants.Field.Width / 2.0, 0),
            robocup.Point(constants.Field.Width / 2.0, constants.Field.Length))

        # handle the case when the ball is near the field's edge
        field_edge_thresh = 0.3
        behind_line = robocup.Segment(main.ball().pos - self._target_line.delta().normalized() * LineKick.DriveAroundDist,
            main.ball().pos - self._target_line.delta().normalized())
        main.system_state().draw_line(behind_line, constants.Colors.Blue, "LineKick")
        for edge in [left_field_edge, right_field_edge]:
            if edge.near_point(main.ball().pos, field_edge_thresh):
                intersection = behind_line.line_intersection(edge)
                if intersection != None:
                    move_goal = intersection

        self.robot.set_avoid_ball_radius(LineKick.SetupBallAvoid)
        self.robot.move_to(move_goal)
        self.robot.face(self.robot.pos + (self.target_point - main.ball().pos))
        self.robot.unkick()


    def execute_charge(self):
        main.system_state().draw_line(robocup.Line(self.robot.pos, self.target_point), constants.Colors.White, "LineKick")
        main.system_state().draw_line(robocup.Line(main.ball().pos, self.target_point), constants.Colors.White, "LineKick")

        # drive directly into the ball
        ball2target = (self.target_point - main.ball().pos).normalized()
        robot2ball = (main.ball().pos, self.robot.pos).normalized()
        speed = min(self.robot.vel.mag() + LineKick.AccelBias, MaxChargeSpeed)
        self.robot.set_world_vel(robot2ball.normalized() * speed)
        
        self.robot.face(self.target_point)

        if self.use_chipper:
            self.robot.chip(self.chip_power)
        else:
            self.kick(self.kick_power)
