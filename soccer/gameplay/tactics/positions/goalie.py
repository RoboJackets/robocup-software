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

class Goalie(single_robot_composite_behavior.SingleRobotCompositeBehavior):

    MaxX = constants.Field.GoalWidth / 2.0
    RobotSegment = robocup.Segment(robocup.Point(-MaxX, constants.Robot.Radius),
                                    robocup.Point(MaxX, constants.Robot.Radius))
    OpponentFacingThreshold = math.pi / 8.0

    class State(enum.Enum):
        ## Normal gameplay, stay towards the side of the goal that the ball is on.
        defend = 1
        ## Opponent has a ball and is prepping a shot we should block.
        block = 2
        ## The ball is moving towards our goal and we should catch it.
        intercept = 3
        ## Get the ball out of our defense area.
        clear = 4
        ## Prepare to block the opponent's penalty shot
        setup_penalty = 5
        ## Keep calm and wait for the ball to be valid.
        chill = 6

    def __init__(self):
        super().__init__(continuous=True)

        for substate in Goalie.State:
            self.add_state(substate, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            Goalie.State.chill,
            lambda: True,
            "immediately")

        self.add_transition(Goalie.State.chill,
            Goalie.State.defend,
            lambda: main.ball().valid,
            "ball is valid")

        non_chill_states = [s for s in Goalie.State if s != Goalie.State.chill]

        # if ball is invalid, chill
        for state in non_chill_states:
            self.add_transition(state,
                Goalie.State.chill,
                lambda: not main.ball().valid,
                "ball is invalid")

        for state in non_chill_states:
            self.add_transition(state,
                Goalie.State.setup_penalty,
                lambda: main.game_state().is_their_penalty() and
                        main.game_state().is_setup_state(),
                "setting up for opponent penalty")

        for state in [s2 for s2 in non_chill_states if s2 != Goalie.State.intercept]:
            self.add_transition(state,
                Goalie.State.intercept,
                lambda: evaluation.ball.is_moving_towards_our_goal() and
                        not self.robot_is_facing_our_goal(evaluation.ball.opponent_with_ball()),
                "ball coming towards our goal")

        for state in [s2 for s2 in non_chill_states if s2 != Goalie.State.clear]:
            self.add_transition(state,
                Goalie.State.clear,
                lambda: evaluation.ball.is_in_our_goalie_zone() and
                        not main.game_state().is_their_penalty() and
                        not evaluation.ball.is_moving_towards_our_goal() and
                        evaluation.ball.opponent_with_ball() is None,
                "ball in our goalie box, but not headed toward goal")

        for state in [s2 for s2 in non_chill_states if s2 != Goalie.State.defend]:
            self.add_transition(state,
                Goalie.State.defend,
                lambda: not evaluation.ball.is_in_our_goalie_zone() and
                        not evaluation.ball.is_moving_towards_our_goal() and
                        not main.game_state().is_their_penalty() and
                        not self.robot_is_facing_our_goal(evaluation.ball.opponent_with_ball()),
                'not much going on')

        for state in [s2 for s2 in non_chill_states if s2 != Goalie.State.block]:
            self.add_transition(state,
                Goalie.State.block,
                lambda: not evaluation.ball.is_in_our_goalie_zone() and
                        not evaluation.ball.is_moving_towards_our_goal() and
                        self.robot_is_facing_our_goal(evaluation.ball.opponent_with_ball()),
                "opponents have possession")


    def robot_is_facing_our_goal(self, robot):
        if robot is None:
            return False
        goal_robot = robot.pos - robocup.Point(0,0)
        angle = goal_robot.normalized().angle() - math.pi
        robot_angle = robot.angle * math.pi / 180.
        self.robot.add_text(str(angle - robot_angle), (255,255,255), "OurRobot")
        if abs(angle - robot_angle) < self.OpponentFacingThreshold:
            return True
        else:
            return False


    # note that execute_running() gets called BEFORE any of the execute_SUBSTATE methods gets called
    def execute_running(self):
        if self.robot != None:
            self.robot.face(main.ball().pos)


    def execute_chill(self):
        if self.robot != None:
            self.robot.move_to(robocup.Point(0, constants.Robot.Radius))


    def execute_setup_penalty(self):
        pt = robocup.Point(0, constants.Field.PenaltyDist)
        penalty_kicker = min(main.their_robots(), key=lambda r: (r.pos - pt).mag())
        angle_rad = penalty_kicker.angle
        shot_line = robocup.Line(penalty_kicker.pos, penalty_kicker.pos + robocup.Point.direction(angle_rad))

        dest = shot_line.line_intersection(Goalie.RobotSegment)
        if dest == None:
            self.robot.move_to(robocup.Point(0, constants.Robot.Radius))
        else:
            dest.x = max(-Goalie.MaxX + constants.Robot.Radius, dest.x)
            dest.x = min(Goalie.MaxX - constants.Robot.Radius, dest.x)
        self.robot.move_to(dest)


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

        kick.target = robocup.Segment(robocup.Point(-constants.Field.Width/2, constants.Field.Length),
            robocup.Point(constants.Field.Width/2, constants.Field.Length))

        # FIXME: if the goalie has a fault, resort to bump

        self.add_subbehavior(kick, 'kick-clear', required=True)


    def on_exit_clear(self):
        self.remove_subbehavior('kick-clear')


    def on_enter_intercept(self):
        i = skills.intercept.Intercept()
        self.add_subbehavior(i, 'intercept', required=True)

    def execute_intercept(self):
        ball_path = robocup.Segment(main.ball().pos,
                            main.ball().pos + main.ball().vel.normalized()*10.0)
        dest = ball_path.nearest_point(self.robot.pos)
        self.robot.move_to(dest)

    def on_exit_intercept(self):
        self.remove_subbehavior('intercept')

    def execute_block(self):
        opposing_kicker = evaluation.ball.opponent_with_ball()
        if opposing_kicker is not None:
            winEval = robocup.WindowEvaluator(main.system_state())
            winEval.excluded_robots = [self.robot]
            best = winEval.eval_pt_to_our_goal(main.ball().pos)[1]
            if best is not None:
                shot_line = robocup.Line(opposing_kicker.pos, main.ball().pos)
                block_line = robocup.Line(robocup.Point(best.segment.get_pt(0).x - constants.Robot.Radius, constants.Robot.Radius),
                                    robocup.Point(best.segment.get_pt(1).x + constants.Robot.Radius, constants.Robot.Radius))
                main.system_state().draw_line(block_line, (255,0,0), "Debug")
                dest = block_line.line_intersection(shot_line)
                dest.x = min(Goalie.MaxX, dest.x)
                dest.x = max(-Goalie.MaxX, dest.x)
                self.robot.move_to(dest)
                return
        self.robot.move_to(robocup.Point(0,constants.Robot.Radius))


    def execute_defend(self):
        dest_x = main.ball().pos.x / constants.Field.Width * Goalie.MaxX
        self.robot.move_to(robocup.Point(dest_x, constants.Robot.Radius))


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
