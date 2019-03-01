import main
import robocup
import math
import behavior
import single_robot_behavior

class Collect(single_robot_behavior.SingleRobotBehavior):
    MAX_INIT_ANGLE_CHANGE = 20 * math.pi/180
    RESTART_MIN_DIST = 0.12
    STOP_SPEED = 0.1

    DRIBBLE_SPEED = 100

    # How many of the last X cycles "has_ball()" was true
    PROBABLY_HELD_MAX = 150

    # How many cycles we want held
    PROBABLY_HELD_CUTOFF = 50

    def __init__(self):
        super().__init__(continuous=False)

        self.probably_held_cnt = 0

        self.old_ball_heading = main.ball().vel

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running,
                            lambda: True, 'immediately')

        # TODO: Add short timeout

        # Only restart if the ball is reasonably far away and 
        # it changed directions or it's moving away from us
        self.add_transition(behavior.Behavior.State.running,
                            behavior.Behavior.State.running,
                            lambda: self.robot is not None and
                                    not self.robot.has_ball() and
                                    (self.robot.pos - main.ball().pos).mag() > Collect.RESTART_MIN_DIST and
                                    self.ball_changed(), 'restart')

        # Complete when we have the ball and it's stopped
        self.add_transition(behavior.Behavior.State.running,
                            behavior.Behavior.State.completed,
                            lambda: self.robot is not None and
                                    self.robot.has_ball() and
                                    self.robot.vel.mag() < Collect.STOP_SPEED and
                                    self.probably_held_cnt > Collect.PROBABLY_HELD_CUTOFF,
                            'ball collected')

        self.add_transition(behavior.Behavior.State.completed,
                            behavior.Behavior.State.running,
                            lambda: self.robot is not None and
                                    (self.probably_held_cnt < Collect.PROBABLY_HELD_CUTOFF or
                                     (self.robot.pos - main.ball().pos).mag() > Collect.RESTART_MIN_DIST),
                            'ball lost')

    def ball_changed(self):
        # Make sure the ball hasn't changed direction significantly
        if (self.old_ball_heading.angle_between(main.ball().vel) > Collect.MAX_INIT_ANGLE_CHANGE):
            return True

        # Make sure we didn't miss the ball
        if (self.robot is not None):
            small_dt = 1.0/20.0
            robot_future_pos = self.robot.pos + self.robot.vel*small_dt
            ball_future_pos = main.ball().pos + main.ball().vel*small_dt

            # If they are moving away
            if ((robot_future_pos - ball_future_pos).mag() > 
                    (self.robot.pos - main.ball().pos).mag()):
                return True

            # Make sure both the ball and robot didn't stop
            if (self.robot.vel.mag() < Collect.STOP_SPEED and
                main.ball().vel.mag() < Collect.STOP_SPEED):
                return True

        return False

    def execute_running(self):
        if (self.robot is not None):
            self.robot.set_dribble_speed(Collect.DRIBBLE_SPEED)
            self.robot.disable_avoid_ball()
            self.robot.collect()

            self.update_held_cnt()

            

    def execute_completed(self):
        if (self.robot is not None):
            self.robot.set_dribble_speed(Collect.DRIBBLE_SPEED)
            self.robot.disable_avoid_ball()

            self.update_held_cnt()

    def update_held_cnt(self):
        # If we see the ball, increment up to max
        # if not, drop to 0
        if (self.robot.has_ball()):
            self.probably_held_cnt = min(self.probably_held_cnt + 1,
                                            Collect.PROBABLY_HELD_MAX)
        else:
            self.probably_held_cnt = max(self.probably_held_cnt - 1, 0)

    def role_requirements(self):
        reqs = super().role_requirements()
        reqs.require_kicking = True
        # try to be near the ball
        if main.ball().valid:
            reqs.destination_shape = main.ball().pos

        return reqs