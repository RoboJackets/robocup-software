import single_robot_behavior
import behavior
import robocup
import main

class Settle(single_robot_behavior.SingleRobotBehavior):

    DRIBBLE_SPEED = 254

    def __init__(self):
        super().__init__(continuous=False)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

    def execute_running(self):
        if(self.robot is not None):
            self.robot.set_dribble_speed(Settle.DRIBBLE_SPEED)
            self.robot.settle(robocup.Point(0,0))

    def role_requirements(self):
        reqs = super().role_requirements()
        # try to be near the ball
        if main.ball().valid:
            reqs.destination_shape = robocup.Segment(main.ball().pos, main.ball().pos + main.ball().vel * 10)

        return reqs
