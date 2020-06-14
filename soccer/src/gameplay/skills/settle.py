import single_robot_behavior
import behavior
import robocup
import main
import constants


# Moves in front of a moving ball to intercept it
# When target is set, settle tries to intercept the ball
# while facing the target to either bounce the ball in that
# direction or kick on breakbeam in that direction
class Settle(single_robot_behavior.SingleRobotBehavior):
    def __init__(self, target=None):
        super().__init__(continuous=False)

        self.target = target
        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

    def execute_running(self):
        if (self.robot is not None):
            self.robot.disable_avoid_ball()
            self.robot.set_dribble_speed(constants.Robot.Dribbler.MaxPower)

            if (self.target is None):
                self.robot.settle()
            else:
                self.robot.settle_w_bounce(self.target)
