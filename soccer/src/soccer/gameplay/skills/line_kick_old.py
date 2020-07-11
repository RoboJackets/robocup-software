import skills._kick
import behavior
import constants
import robocup
import enum
import main
import role_assignment


## lines up with the ball and the target, then drives up and kicks
# this differs from PivotKick which gets the ball first, then aims
# Note: LineKickOld recalculates the target_aim_point ONLY when the target point/segment changes
#
# LineKickOld is a version of LineKick implemented completely in python
# The current version of LineKick is implemented in the C++ to take
# advantage of path planning there.
# While they should produce the same end result, the new LineKick
# approaches the ball quicker and more accurately, while this one
# needs to remain more cautious (as it cannot interact directly with pathplanner)
class LineKickOld(skills._kick._Kick):

    ## tuneable constants
    DefaultDriveAroundDist = 0.15
    ChargeThresh = 0.1
    EscapeChargeThresh = 0.1
    DefaultSetupBallAvoid = 0.15
    AccelBias = 0.2
    FacingThresh = 10  # angle in degrees
    MaxChargeSpeed = 1.5
    BallProjectTime = 0.4
    DoneStateThresh = 0.11
    ClosenessThreshold = constants.Robot.Radius + 0.04

    class State(enum.Enum):
        setup = 1
        charge = 2

    def __init__(self):
        super().__init__()

        self._got_close = False

        self.setup_ball_avoid = LineKickOld.DefaultSetupBallAvoid
        self.drive_around_dist = LineKickOld.DefaultDriveAroundDist

        for state in LineKickOld.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            LineKickOld.State.setup, lambda: True,
                            'immediately')

        self.add_transition(
            LineKickOld.State.setup, LineKickOld.State.charge,
            lambda: self.enable_kick and (not self.robot_is_between_ball_and_target()) and self._target_line.dist_to(self.robot.pos) < self.ChargeThresh and not self.robot.just_kicked(),
            "robot on line")

        self.add_transition(
            LineKickOld.State.charge, behavior.Behavior.State.completed,
            lambda: self.robot is not None and self._got_close and self.robot.just_kicked(),
            "robot kicked")

        self.add_transition(
            LineKickOld.State.charge, LineKickOld.State.setup,
            lambda: self.robot_is_between_ball_and_target() or self._target_line.dist_to(self.robot.pos) > self.ChargeThresh,
            "robot between ball and target")

    ## We use the path planner to move to a point this distance away from the
    # ball and on the opposite side of where we're aiming
    @property
    def drive_around_dist(self):
        return self._drive_around_dist

    @drive_around_dist.setter
    def drive_around_dist(self, value):
        self._drive_around_dist = value

    ## The avoid ball radius for while we're getting to the setup point
    @property
    def setup_ball_avoid(self):
        return self._setup_ball_avoid

    @setup_ball_avoid.setter
    def setup_ball_avoid(self, value):
        self._setup_ball_avoid = value

    def robot_is_between_ball_and_target(self):
        return self.robot is not None and \
            self.robot.pos.dist_to(self.aim_target_point) < main.ball().pos.dist_to(self.aim_target_point)

    def recalculate(self):
        self._target_line = robocup.Line(main.ball().pos,
                                         self.aim_target_point)
        # FIXME: errors?

    def on_exit_start(self):
        super().recalculate_aim_target_point()

    def execute_running(self):
        self.recalculate()
        super().execute_running()

    def execute_setup(self):
        move_goal = main.ball().pos - self._target_line.delta().normalized(
        ) * (self.drive_around_dist + constants.Robot.Radius)

        left_field_edge = robocup.Segment(
            robocup.Point(
                -constants.Field.Width / 2.0 - constants.Robot.Radius, 0),
            robocup.Point(
                -constants.Field.Width / 2.0 - constants.Robot.Radius,
                constants.Field.Length))
        right_field_edge = robocup.Segment(
            robocup.Point(constants.Field.Width / 2.0 + constants.Robot.Radius,
                          0),
            robocup.Point(constants.Field.Width / 2.0 + constants.Robot.Radius,
                          constants.Field.Length))

        # handle the case when the ball is near the field's edge
        field_edge_thresh = 0.3
        behind_line = robocup.Segment(
            main.ball().pos - self._target_line.delta().normalized() *
            self.drive_around_dist,
            main.ball().pos - self._target_line.delta().normalized())
        main.debug_drawer().draw_line(behind_line, constants.Colors.Blue,
                                      "LineKickOld")
        for edge in [left_field_edge, right_field_edge]:
            if edge.near_point(main.ball().pos, field_edge_thresh):
                intersection = behind_line.segment_intersection(edge)
                if intersection != None:
                    move_goal = intersection

        self.robot.set_avoid_ball_radius(self.setup_ball_avoid)
        self.robot.move_to(move_goal)
        self.robot.face(self.robot.pos + (self.aim_target_point - main.ball(
        ).pos))
        self.robot.unkick()

    def execute_charge(self):
        self.robot.disable_avoid_ball()
        main.debug_drawer().draw_line(
            robocup.Line(self.robot.pos, self.aim_target_point),
            constants.Colors.White, "LineKickOld")
        main.debug_drawer().draw_line(
            robocup.Line(main.ball().pos, self.aim_target_point),
            constants.Colors.White, "LineKickOld")

        # drive directly into the ball
        ball2target = (self.aim_target_point - main.ball().pos).normalized()
        robot2ball = (main.ball().pos - self.robot.pos).normalized()
        speed = min(self.robot.vel.mag() + LineKickOld.AccelBias,
                    self.MaxChargeSpeed)
        self.robot.set_world_vel(robot2ball.normalized() * speed)

        self.robot.face(self.aim_target_point)

        if main.ball().pos.dist_to(
                self.robot.pos) < LineKickOld.ClosenessThreshold:
            self._got_close = True

        if self.robot.has_ball():
            if self.use_chipper:
                self.robot.chip(self.chip_power)
            else:
                self.robot.kick(self.kick_power)

    def role_requirements(self):
        reqs = super().role_requirements()
        # try to be near the ball
        if main.ball().valid:
            reqs.destination_shape = main.ball().pos
        reqs.require_kicking = True
        if self.use_chipper:
            reqs.chipper_preference_weight = role_assignment.PreferChipper
        return reqs
