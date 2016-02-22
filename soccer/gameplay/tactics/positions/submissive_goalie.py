import robocup
import single_robot_composite_behavior
import behavior
import role_assignment
import constants
import evaluation.ball
import skills
import main
import enum
import math
import evaluation


# The regular goalie does a lot of calculations and figures out where it should be
# This goalie lets someone else (the Defense tactic) handle calculations and blocks things based on that
# TODO: merge this back into the regular goalie?
class SubmissiveGoalie(
        single_robot_composite_behavior.SingleRobotCompositeBehavior):

    MaxX = constants.Field.GoalWidth / 2.0
    SegmentY = constants.Robot.Radius + 0.05

    # The segment we stay on during the 'block' state
    # It's right in front of the goal
    RobotSegment = robocup.Segment(
        robocup.Point(-MaxX, SegmentY), robocup.Point(MaxX, SegmentY))

    class State(enum.Enum):
        "Actively blocking based on a given threat"
        block = 2
        "The ball is moving towards our goal and we should catch it."
        intercept = 3
        "Get the ball out of our defense area."
        clear = 4

    def __init__(self):
        super().__init__(continuous=True)

        for substate in SubmissiveGoalie.State:
            self.add_state(substate, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            SubmissiveGoalie.State.block, lambda: True,
                            "immediately")

        non_block_states = [s
                            for s in SubmissiveGoalie.State
                            if s != SubmissiveGoalie.State.block]

        for state in [s2
                      for s2 in SubmissiveGoalie.State
                      if s2 != SubmissiveGoalie.State.intercept]:
            self.add_transition(
                state, SubmissiveGoalie.State.intercept,
                lambda: evaluation.ball.is_moving_towards_our_goal(),
                "ball coming towards our goal")

        for state in [s2
                      for s2 in SubmissiveGoalie.State
                      if s2 != SubmissiveGoalie.State.clear]:
            self.add_transition(
                state, SubmissiveGoalie.State.clear,
                lambda: evaluation.ball.is_in_our_goalie_zone() and not evaluation.ball.is_moving_towards_our_goal() and main.ball().vel.mag() < 0.4 and evaluation.ball.opponent_with_ball() is None,
                "ball in our goalie box, but not headed toward goal")

        for state in non_block_states:
            self.add_transition(
                state, SubmissiveGoalie.State.block,
                lambda: not evaluation.ball.is_in_our_goalie_zone() and not evaluation.ball.is_moving_towards_our_goal(),
                'ball not in goal or moving towards it')

        self.block_line = None
        self._move_target = robocup.Point(0, 0)

    # the line we expect a threat to shoot from
    # sits on the intersection of this line and the goalie segment
    @property
    def block_line(self):
        return self._block_line

    @block_line.setter
    def block_line(self, value):
        self._block_line = value

        if self.block_line == None:
            self._move_target = SubmissiveGoalie.RobotSegment.center()
        else:
            self._move_target = SubmissiveGoalie.RobotSegment.nearest_point_to_line(
                self.block_line)

        self._move_target.x = min(
            max(self._move_target.x,
                -SubmissiveGoalie.MaxX), SubmissiveGoalie.MaxX)

    # The point we'll be going to in order to block the given block_line
    @property
    def move_target(self):
        return self._move_target

    # note that execute_running() gets called BEFORE any of the execute_SUBSTATE methods gets called
    def execute_running(self):
        self.robot.face(main.ball().pos)

    def on_enter_clear(self):
        # FIXME: what we really want is a less-precise LineKick
        #           this will require a Capture behavior that doesn't wait for the ball to stop
        kick = skills.pivot_kick.PivotKick()

        # TODO: the below dribble speed is best for a 2011 bot
        # kick.dribble_speed = constants.Robot.Dribbler.MaxPower / 3.5

        # we use low error thresholds here
        # the goalie isn't trying to make a shot, he just wants get the ball the **** out of there
        kick.aim_params['error_threshold'] = 1.0
        kick.aim_params['max_steady_ang_vel'] = 12

        # chip
        kick.chip_power = 1.0
        kick.use_chipper = True

        kick.target = robocup.Segment(
            robocup.Point(-constants.Field.Width / 2, constants.Field.Length),
            robocup.Point(constants.Field.Width / 2, constants.Field.Length))

        # FIXME: if the goalie has a fault, resort to bump

        self.add_subbehavior(kick, 'kick-clear', required=True)

    def on_exit_clear(self):
        self.remove_subbehavior('kick-clear')

    def on_enter_intercept(self):
        i = skills.intercept.Intercept()
        i.shape_constraint = SubmissiveGoalie.RobotSegment
        self.add_subbehavior(i, 'intercept', required=True)

    def on_exit_intercept(self):
        self.remove_subbehavior('intercept')

    def on_enter_block(self):
        move = skills.move.Move()
        self.add_subbehavior(move, 'move', required=True)

    def execute_block(self):
        move = self.subbehavior_with_name('move')
        move.pos = self.move_target

    def on_exit_block(self):
        self.remove_subbehavior('move')

    def role_requirements(self):
        reqs = super().role_requirements()

        for req in role_assignment.iterate_role_requirements_tree_leaves(reqs):
            req.required_shell_id = self.shell_id if self.shell_id != None else -1
        return reqs

    @property
    def shell_id(self):
        return self._shell_id

    @shell_id.setter
    def shell_id(self, value):
        self._shell_id = value
