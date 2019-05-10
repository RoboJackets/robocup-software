import single_robot_behavior
import behavior
import robocup
import main

class Settle(single_robot_behavior.SingleRobotBehavior):

    DRIBBLE_SPEED = 150

    def __init__(self, target=None):
        super().__init__(continuous=False)

        self.target = target
        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

    def execute_running(self):
        if(self.robot is not None):
            self.robot.disable_avoid_ball()
            self.robot.set_dribble_speed(Settle.DRIBBLE_SPEED)

            if (self.target is None):
                self.robot.settle()
            else:
                self.robot.settle_w_bounce(self.target)