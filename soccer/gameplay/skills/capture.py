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

    # We want  to let the probably held count stabilize
    # but to do that we have to let it sit still
    # If it doesn't move to either extreme and stays at 50%
    # we can't transition away, so we need this counter
    MAX_FRAMES_IN_CAPTURED = 20

    # X number of frames in the past to look at
    PROBABLY_HELD_HISTORY_LENGTH = 60

    PROBABLY_HELD_START = .5 * PROBABLY_HELD_HISTORY_LENGTH

    # Anything below this number we think we don't have the ball
    PROBABLY_NOT_HELD_CUTOFF = .4 * PROBABLY_HELD_HISTORY_LENGTH
    # Anything above this number we think we have the ball
    PROBABLY_HELD_CUTOFF = .6 * PROBABLY_HELD_HISTORY_LENGTH

    class State(Enum):
        # May already have the ball so just sit for a split second and keep dribbler on
        captured = 0

        # Don't have ball so try to intercept the moving ball
        settle = 2

        # Don't have ball so try to do the fine approach to control the ball
        collect = 1

    ## Capture Constructor
    # faceBall - If false, any turning functions are turned off,
    # useful for using capture to reflect/bounce moving balls.
    def __init__(self, faceBall=True):
        super().__init__(continuous=False)

        for s in Capture.State:
            self.add_state(s, behavior.Behavior.State.running)

        self.prev_ball = main.ball()
        self.probably_held_cnt = Capture.PROBABLY_HELD_START
        self.frames_in_captured = 0

        
        # By default, move into the settle state since we almost never start with ball
        self.add_transition(behavior.Behavior.State.start,
                            Capture.State.settle,
                            lambda: self.robot is not None and not self.robot.has_ball(),
                            'dont have ball')

        # On the offchance we start with ball, double check it's not a blip on the sensor
        self.add_transition(behavior.Behavior.State.start,
                            Capture.State.captured,
                            lambda: self.robot is not None and self.robot.has_ball(),
                            'may already have ball')

        # We actually don't have the ball, either 50% register rate (faulty sensor?) or we just got a blip
        self.add_transition(Capture.State.captured,
                            Capture.State.settle,
                            lambda: self.probably_held_cnt < Capture.PROBABLY_NOT_HELD_CUTOFF or
                                    self.frames_in_captured > Capture.MAX_FRAMES_IN_CAPTURED,
                            'actually dont have ball')

        # Actually do have the ball, we can just leave
        self.add_transition(Capture.State.captured,
                            behavior.Behavior.State.completed,
                            lambda: self.probably_held_cnt > Capture.PROBABLY_HELD_CUTOFF,
                            'actually have ball')

        # If we intercepted the ball to slow it down enough or it most likely just bounced off our robot and
        # The velocity is just funky
        self.add_transition(Capture.State.settle,
                            Capture.State.collect,
                            lambda: main.ball().vel.mag() < Capture.INTERCEPT_VELOCITY_THRESH, # or
                                    #self.ball_bounced_off_robot() and not self.ball_probably_kicked(),
                            'collecting')

        # Cut back if the velocity is pretty high or it was probably kicked
        self.add_transition(Capture.State.collect,
                            Capture.State.settle,
                            lambda: main.ball().vel.mag() >= Capture.INTERCEPT_VELOCITY_THRESH and
                                    (self.subbehavior_with_name('collector').robot is not None and
                                     (self.subbehavior_with_name('collector').robot.pos - main.ball().pos).mag() > .3), # or 
                                    #self.ball_probably_kicked() and not self.ball_bounced_off_robot(),
                            'settling again')

        #  Once collect is done, we have the ball
        self.add_transition(Capture.State.collect,
                            behavior.Behavior.State.completed,
                            lambda: self.subbehavior_with_name('collector').is_done_running(),
                            'captured')

        # Go back to settle if we loose the ball when completed
        self.add_transition(behavior.Behavior.State.completed,
                            Capture.State.settle,
                            lambda: self.probably_held_cnt < Capture.PROBABLY_NOT_HELD_CUTOFF,
                            'captured')


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
        # Check the angle between is less than 90 degrees
        # On the unit circle, two vectors are within 90 degrees of each other if the distance
        # between two vectors is less than sqrt(1^2 + 1^2)
        #
        #              +1 | a
        #                 |
        #                 |
        #                 |            b
        #                 |_____________
        #                              +1
        #
        # Dist between a and b are sqrt(2)
        #
        #
        #     a                         b
        #     ____________________________
        #     -1                        +1
        #
        # Dist between a and b is 2
        return (vec1.norm() - vec2.norm()).mag() < math.sqrt(2)

    def on_enter_captured(self):
        print("Enter captured")

    def execute_captured(self):
        self.update_held_cnt()
        self.frames_in_captured += 1

    def on_enter_settle(self):
        print("Enter settle")
        self.remove_all_subbehaviors()
        self.add_subbehavior(skills.settle.Settle(),
                             name='settler',
                             required=True,
                             priority=10)

    def on_enter_collect(self):
        print("Enter collect")
        self.remove_all_subbehaviors()
        self.add_subbehavior(skills.collect.Collect(),
                             name='collector',
                             required=True,
                             priority=10)

    def on_enter_completed(self):
        self.probably_held_cnt = Capture.PROBABLY_HELD_START

    def execute_completed(self):
        self.update_held_cnt()

    def update_held_cnt(self):
        if (self.robot is None):
            return

        # If we hold the ball, increment up to max
        # if not, drop to 0
        if (self.robot.has_ball()):
        #if (self.robot is not None and self.robot.vel.mag() < Collect.STOP_SPEED):
            self.probably_held_cnt = min(self.probably_held_cnt + 1, Capture.PROBABLY_HELD_HISTORY_LENGTH)
        else:
            self.probably_held_cnt = max(self.probably_held_cnt - 1, 0)

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