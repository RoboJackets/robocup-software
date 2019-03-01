import single_robot_composite_behavior
import behavior
from enum import Enum
import main
import role_assignment
import robocup
import constants
import skills.settle
import skills.collect
import math



class Capture(single_robot_composite_behavior.SingleRobotCompositeBehavior):

    INTERCEPT_VELOCITY_THRESH = 0.5

    PROBABLE_KICK_CHANGE = 0.1

    class State(Enum):
        settle = 0
        collect = 1

    ## Capture Constructor
    # faceBall - If false, any turning functions are turned off,
    # useful for using capture to reflect/bounce moving balls.
    def __init__(self, faceBall=True):
        super().__init__(continuous=False)

        self.add_state(Capture.State.settle,
                       behavior.Behavior.State.running)
        self.add_state(Capture.State.collect,
                       behavior.Behavior.State.running)

        self.prev_ball = main.ball()

        self.add_transition(behavior.Behavior.State.start,
                            Capture.State.settle, lambda: True,
                            'immediately')

        self.add_transition(Capture.State.settle,
                            Capture.State.collect,
                            lambda: main.ball().vel.mag() < Capture.INTERCEPT_VELOCITY_THRESH or
                                    self.ball_bounced_off_robot(),
                            'collecting')

        self.add_transition(Capture.State.collect,
                            behavior.Behavior.State.completed,
                            lambda: self.subbehavior_with_name('collector').is_done_running(),
                            'captured')

        # Cut back if the velocity is pretty high and it was probably kicked
        self.add_transition(Capture.State.collect,
                            Capture.State.settle,
                            lambda: main.ball().vel.mag() >= Capture.INTERCEPT_VELOCITY_THRESH and 
                                    self.ball_probably_kicked(),
                            'settling again')

    def ball_bounced_off_robot(self):
        # Check if ball is near current robot
        # Check if the ball velocity went from coming to us
        #   to moving away
        if (self.robot is not None):
            old_ball_to_bot = self.robot.pos - self.prev_ball.pos
            ball_to_bot = self.robot.pos - main.ball().pos

            # If the old ball is not in the same direction as the current one
            is_bounce = (self.is_same_direction(old_ball_to_bot, self.prev_ball.vel) and
                         not self.is_same_direction(ball_to_bot, main.ball().vel))

            # And we are near the ball
            near_robot = ball_to_bot.mag() < 2*constants.Robot.Radius

            return is_bounce and near_robot

        else:
            return False

    def ball_probably_kicked(self):
        if (self.robot is not None):
            # Check if there is a significant jump in velocity
            prob_kick = (self.prev_ball.vel - main.ball().vel).mag() > Capture.PROBABLE_KICK_CHANGE

            # And we are not near the ball
            near_robot = (self.robot.pos - main.ball().pos).mag() < 2*constants.Robot.Radius

            return prob_kick and not near_robot
        else:
            return False

    def is_same_direction(self, vec1, vec2):
        angle = vec1.angle_between(vec2)
        return angle < math.pi/2

    def on_enter_settle(self):
        self.remove_all_subbehaviors()
        self.add_subbehavior(skills.settle.Settle(),
                             name='settler',
                             required=True,
                             priority=10)

    def on_enter_collect(self):
        self.remove_all_subbehaviors()
        self.add_subbehavior(skills.collect.Collect(),
                             name='collector',
                             required=True,
                             priority=10)

    def role_requirements(self):
        reqs = super().role_requirements()
        # reqs.require_kicking = True
        # try to be near the ball
        # if main.ball().valid:
        #     if main.ball().vel.mag() < Capture.INTERCEPT_VELOCITY_THRESH:
        #         reqs.destination_shape = main.ball().pos
        #     else:
        #         # TODO Make this less complicated and remove magic numbers
        #         reqs.destination_shape = robocup.Segment(main.ball().pos, main.ball().pos + main.ball().vel * 10)
        return reqs