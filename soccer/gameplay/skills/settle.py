import single_robot_behavior
import behavior
import robocup
import main

class Settle(single_robot_behavior.SingleRobotBehavior):
    def __init__(self):
        super().__init__(continuous=False)

        self.speedThreshold = 0.02

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        self.add_transition(
            behavior.Behavior.State.running, behavior.Behavior.State.completed,
            lambda: main.ball().vel.mag() < self.speedThreshold,
            'Settled')
        self.add_transition(
            behavior.Behavior.State.completed, behavior.Behavior.State.running,
            lambda: main.ball().vel.mag() > self.speedThreshold,
            'Ball moving')

    def execute_running(self):
        if(self.robot is not None):
            self.robot.settle(robocup.Point(0,0))

    def role_requirements(self):
        reqs = super().role_requirements()
        reqs.require_kicking = True
        # try to be near the ball
        if main.ball().valid:
            reqs.destination_shape = main.ball().pos

        return reqs
