import single_robot_behavior
import behavior
from enum import Enum
import main
import evaluation
import constants
import role_assignment


class Capture(single_robot_behavior.SingleRobotBehavior):

    # tunable config values
    CourseApproachErrorThresh = 0.8
    CourseApproachDist = 0.18
    CourseApproachAvoidBall = 0.10
    DribbleSpeed = 100
    FineApproachSpeed = 0.2


    class State(Enum):
        course_approach = 1
        fine_approach = 2


    def __init__(self):
        super().__init__(continuous=False)

        self.add_state(Capture.State.course_approach, behavior.Behavior.State.running)
        self.add_state(Capture.State.fine_approach, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            Capture.State.course_approach,
            lambda: True,
            'immediately')

        self.add_transition(Capture.State.course_approach,
            Capture.State.fine_approach,
            lambda: self.bot_in_approach_pos() and main.ball().valid,
            'dist to ball < threshold')

        self.add_transition(Capture.State.fine_approach,
            behavior.Behavior.State.completed,
            lambda: self.robot.has_ball(),
            'has ball')

        self.add_transition(Capture.State.fine_approach,
            Capture.State.course_approach,
            lambda: main.ball().pos.dist_to(self.robot.pos) > Capture.CourseApproachDist * 1.5 or not main.ball().valid,
            'ball ran away')


    def bot_in_approach_pos(self):
        bot2ball = main.ball().pos - self.robot.pos
        ball2bot = bot2ball * -1
        approach_vec = self.approach_vector()

        return (ball2bot.normalized().dot(approach_vec) > Capture.CourseApproachErrorThresh and
                bot2ball.mag() < Capture.CourseApproachDist * 1.5)

    # normalized vector pointing from the ball to the point the robot should get to in course_aproach
    def approach_vector(self):
        if main.ball().vel.mag() > 0.25 and self.robot.pos.dist_to(main.ball().pos) > 0.2:
            # ball's moving, get on the side it's moving towards
            return main.ball().vel.normalized()
        else:
            return (self.robot.pos - main.ball().pos).normalized()


    def find_intercept_point(self):
        approach_vec = self.approach_vector()

        # sample every 5 cm in the -approach_vector direction from the ball
        pos = None
        for i in range(50):
            dist = i * 0.05
            pos = main.ball().pos + approach_vec * dist
            ball_time = evaluation.ball.rev_predict(main.ball().vel, dist - Capture.CourseApproachDist) # how long will it take the ball to get there
            bot_time = (pos - self.robot.pos).mag() * 10.0 # FIXME: evaluate trapezoid

            # print('bot: ' + str(bot_time) + ';; ball: ' + str(ball_time))

            if bot_time < ball_time:
                break

        return pos


    def execute_running(self):
        # make sure teammates don't bump into us
        self.robot.shield_from_teammates(constants.Robot.Radius * 2.0)


    def execute_course_approach(self):
        # don't hit the ball on accident
        self.robot.set_avoid_ball_radius(Capture.CourseApproachAvoidBall)
        pos = self.find_intercept_point()
        self.robot.face(main.ball().pos)
        if pos != None and main.ball().valid:
            main.system_state().draw_circle(pos, constants.Ball.Radius, constants.Colors.White, "Capture")
            self.robot.move_to(pos)


    def execute_fine_approach(self):
        self.robot.disable_avoid_ball()
        self.robot.set_dribble_speed(Capture.DribbleSpeed)
        self.robot.face(main.ball().pos)

        bot2ball = (main.ball().pos - self.robot.pos).normalized()
        self.robot.set_world_vel(bot2ball * Capture.FineApproachSpeed + main.ball().vel)


    def role_requirements(self):
        reqs = super().role_requirements()

        for r in role_assignment.iterate_role_requirements_tree_leaves(reqs):
            # try to be near the ball
            if main.ball().valid:
                r.destination_shape = main.ball().pos
            r.require_kicking = True

        return reqs
