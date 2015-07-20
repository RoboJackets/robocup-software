import single_robot_behavior
import behavior
from enum import Enum
import main
import evaluation
import constants
import robocup

class Capture(single_robot_behavior.SingleRobotBehavior):

    # tunable config values
    CourseApproachErrorThresh = 0.8
    CourseApproachDist = 0.3
    CourseApproachAvoidBall = 0.10
    DribbleSpeed = 100
    FineApproachSpeed = 0.2
    BackOffDistance = 0.4
    BackOffSpeed = 0.3


    class State(Enum):
        course_approach = 1
        fine_approach = 2
        back_off = 3


    def __init__(self):
        super().__init__(continuous=False)

        self.add_state(Capture.State.course_approach, behavior.Behavior.State.running)
        self.add_state(Capture.State.fine_approach, behavior.Behavior.State.running)
        self.add_state(Capture.State.back_off, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            Capture.State.course_approach,
            lambda: True,
            'immediately')

        self.add_transition(Capture.State.course_approach,
            Capture.State.fine_approach,
            lambda: self.bot_near_ball(Capture.CourseApproachDist) and main.ball().valid and not constants.Field.TheirGoalShape.contains_point(main.ball().pos),
            'dist to ball < threshold')

        self.add_transition(Capture.State.fine_approach,
            behavior.Behavior.State.completed,
            lambda: self.robot.has_ball(),
            'has ball')

        self.add_transition(Capture.State.fine_approach,
            Capture.State.course_approach,
            lambda: not self.robot.has_ball() and (not self.bot_near_ball(Capture.CourseApproachDist * 1.5) or not main.ball().pos),
            'ball went into goal')

        self.add_transition(Capture.State.fine_approach,
            Capture.State.back_off,
            lambda: constants.Field.TheirGoalShape.contains_point(main.ball().pos),
            'ball ran away')

        self.add_transition(Capture.State.back_off,
            behavior.Behavior.State.start,
            lambda: bot_to_ball().mag()<BackOffDistance,
            "backed away enough")


        self.lastApproachTarget = None
        self.postChangeCount = 0

    def bot_to_ball(self):
        return main.ball().pos - self.robot.pos

    def bot_near_ball(self, distance):
        return (self.bot_to_ball().mag() < distance)

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
        for i in range(100):
            dist = i * 0.05
            pos = main.ball().pos + approach_vec * dist
            ball_time = evaluation.ball.rev_predict(main.ball().vel, dist) # how long will it take the ball to get there
            
            robotDist = (pos - self.robot.pos).mag()*0.9
            bot_time = robocup.get_trapezoidal_time(
                robotDist, 
                robotDist,
                2.2,
                1,
                self.robot.vel.mag(),
                0)
            #bot_time = (pos - self.robot.pos).mag() * 30.0 # FIXME: evaluate trapezoid
            # print('bot: ' + str(bot_time) + ';; ball: ' + str(ball_time))

            if bot_time < ball_time:
                break
            #if i == 50:


        return pos


    def execute_running(self):
        # make sure teammates don't bump into us
        self.robot.shield_from_teammates(constants.Robot.Radius * 2.0)

    def on_enter_course_approach(self):
        self.lastApproachTarget == None
        self.pastChangeCount = 11

    def execute_course_approach(self):
        # don't hit the ball on accident
        self.robot.set_avoid_ball_radius(Capture.CourseApproachAvoidBall)
        pos = self.find_intercept_point()
        self.robot.face(main.ball().pos)

        if self.pastChangeCount <=0 or (self.lastApproachTarget != None and (pos - self.lastApproachTarget).mag()<0.1):
            self.pastChangeCount = self.pastChangeCount + 1
            self.robot.move_to(self.lastApproachTarget)
        else:    
            main.system_state().draw_circle(pos, constants.Ball.Radius, constants.Colors.White, "Capture")
            self.robot.move_to(pos)
            self.lastApproachTarget = pos
            self.pastChangeCount = 0;

    def on_exit_course_approach(self):
       self.lastApproachTarget == None


    def execute_fine_approach(self):
        Capture.multiplier = 1.0

        self.robot.disable_avoid_ball()
        self.robot.set_dribble_speed(Capture.DribbleSpeed)
        self.robot.face(main.ball().pos)

        bot2ball = (main.ball().pos - self.robot.pos).normalized()
        self.robot.set_world_vel(self.bot_to_ball()*Capture.multiplier + bot2ball * Capture.FineApproachSpeed/4 + main.ball().vel)

    def execute_back_off(self):
        self.robot.face(main.ball().pos)
        self.robot.set_world_vel(self.bot_to_ball().normalized() * -1 * Capture.BackOffSpeed)

    def role_requirements(self):
        reqs = super().role_requirements()
        reqs.require_kicking = True
        return reqs