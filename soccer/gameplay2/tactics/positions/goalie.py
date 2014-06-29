import robocup
import single_robot_composite_behavior
import behavior
import constants
import evaluation.ball
import skills
import main
import enum
import math


class Goalie(single_robot_composite_behavior.SingleRobotCompositeBehavior):

    MaxX = constants.Field.GoalWidth / 2.0
    RobotSegment = robocup.Segment(robocup.Point(-MaxX, constants.Robot.Radius),
                                    robocup.Point(MaxX, constants.Robot.Radius))

    class State(enum.Enum):
        defend = 1          # TODO: clarify difference between block and defend
        block = 2
        intercept = 3
        clear = 4           # kick/chip the ball out if it's in our goal zone
        setup_penalty = 5   # get in place for a penalty shot
        chill = 6           # if we can't see the ball, we just sit in the center in front of our goal


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
                lambda: main.game_state().is_their_penalty() and not main.game_state().playing(),
                "opponent penalty shot about to happen")

        for state in non_chill_states:
            self.add_transition(state,
                Goalie.State.intercept,
                lambda: evaluation.ball.is_moving_towards_our_goal(),
                "ball coming towards our goal")

        for state in non_chill_states:
            self.add_transition(state,
                Goalie.State.clear,
                lambda: evaluation.ball.is_in_our_goalie_zone() and
                        not evaluation.ball.is_moving_towards_our_goal(),
                "ball in our goalie box, but not headed toward goal")

        for state in non_chill_states:
            self.add_transition(state,
                Goalie.State.defend,
                lambda: not evaluation.ball.is_in_our_goalie_zone() and
                        not evaluation.ball.is_moving_towards_our_goal() and
                        evaluation.ball.opponent_with_ball() == None,
                'not much going on')



    # note that execute_running() gets called BEFORE any of the execute_SUBSTATE methods gets called
    def execute_running(self):
        if self.robot != None:
            self.robot.face(main.ball().pos)


    def execute_chill(self):
        if self.robot != None:
            self.robot.move_to(robocup.Point(0, constants.Robot.Radius))


    def execute_setup_penalty(self):
        pt = Point(0, Field_PenaltyDist)
        penalty_kicker = min(main.opponent_robots(), key=lambda r: (r.pos - pt).mag())
        angle = math.pi * penalty_kicker.angle / 180.0
        shot_line = Line(penalty_kicker.pos,
                            Point(penalty_kicker.pos.x + math.cos(angle)),)

        dest = shot_line.intersection(Goalie.RobotSegment)
        if dest == None:
            robot.move_to(robocup.Point(0, constants.Robot.Radius))
        else:
            dest.x = max(-Goalie.MaxX + constants.Robot.Radius, dest.x)
            dest.y = min(Goalie.MaxX - constants.Robot.Radius, dest.x)
        robot.move_to(dest)


    # The below is the old C++ code ported to python - instead of using this I've (justin) replaced it with the pivot kick behavior
    # remove this old stuff once we verify that the new behavior works well
    # def execute_clear(self):
    #     ball_to_goal = robocup.Segment(main.ball().pos, Point(0, 0))
    #     closest = ball_to_goal.nearest_point(robot.pos)

    #     if (robot.pos - closest).mag() > 0.10:
    #         robot.move_to(closest)
    #     else:
    #         robot.set_world_vel(main.ball().pos - robot.pos).normalized() * 1.0

    #     robot.dribble(40)
    #     robot.face(main.ball().pos())
    #     robot.unkick()

    #     if robot.has_chipper():
    #         robot.chip(255)
    #     else:
    #         robot.kick(255)

    def on_enter_clear(self):
        # FIXME: what we really want is a less-precise LineKick
        #           this will require a Capture behavior that doesn't wait for the ball to stop
        kick = skills.pivot_kick.PivotKick()
        # FIXME: config pivot kick
        # FIXME: enable chipping!
        # FIXME: if the goalie has a fault, resort to bump
        self.add_subbehavior('kick-clear', kick, required=True)


    def on_exit_clear(self):
        self.remove_behavior('kick-clear')


    def execute_intercept(self):
        ball_path = robocup.Segment(main.ball().pos,
                            main.ball().pos + 10*main.ball().vel.normalized())
        dest = ball_path.nearest_point(robot.pos)
        robot.move_to(dest)


    def execute_block(self):
        opposing_kicker = evaluation.opponent_with_ball()
        shot_line = Line(opposing_kicker.pos, main.ball().pos)
        block_circle = Circle(Point(0, 0), constants.Field.GoalWidth / 2.0)

        dest = block_circle.intersection(shot_line)
        if dest != None:
            robot.move_to(dest)
        else:
            block_line = Line(Point(-Goalie.MaxX, constants.Robot.Radius),
                                Point(Goalie.MaxX, constants.Robot.Radius))
            dest = block_line.intersection(shot_line)
            dest.x = min(Goalie.MaxX, dest.x)
            dest.x = max(-Goalie.MaxX, dest.x)
            robot.move_to(dest)


    def execute_defend(self):
        dest_x = main.ball().pos.x / constants.Field.Width * Goalie.MaxX
        self.robot.move_to(robocup.Point(dest_x, constants.Robot.Radius))


    def role_requirements(self):
        reqs = super().role_requirements()

        if isinstance(reqs, role_requirements.RoleRequirements):
            reqs.required_shell_id = self.shell_id
        else:
            for sub_bhvr_name in reqs:
                reqs[sub_bhvr_name].required_shell_id = self.shell_id

        return reqs


    @property
    def shell_id(self):
        return self._shell_id
    @shell_id.setter
    def shell_id(self, value):
        self._shell_id = value
