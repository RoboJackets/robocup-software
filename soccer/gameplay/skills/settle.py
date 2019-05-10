import single_robot_behavior
import behavior
import robocup
import main

class Settle(single_robot_behavior.SingleRobotBehavior):

    DRIBBLE_SPEED = 90

    def __init__(self):
        super().__init__(continuous=False)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

    def execute_running(self):
        if(self.robot is not None):
            self.robot.disable_avoid_ball()
            self.robot.set_dribble_speed(Settle.DRIBBLE_SPEED)
            self.robot.settle(robocup.Point(0,0))