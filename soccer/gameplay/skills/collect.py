import single_robot_behavior
import behavior
import robocup
import main

class Collect(single_robot_behavior.SingleRobotBehavior):
    def __init__(self):
        super().__init__(continuous=False)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        self.add_transition(behavior.Behavior.State.running,
                            behavior.Behavior.State.completed,
                            lambda: self.robot is not None and
                                    self.robot.has_ball(),
                            'ball collected')

    def execute_running(self):
        if (self.robot is not None):
            self.robot.set_dribble_speed(100)
            self.robot.collect()

    def role_requirements(self):
        reqs = super().role_requirements()
        reqs.require_kicking = True
        # try to be near the ball
        if main.ball().valid:
            reqs.destination_shape = main.ball().pos

        return reqs