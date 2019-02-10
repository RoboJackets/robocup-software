import single_robot_behavior
import behavior
from enum import Enum
import main
import evaluation
import constants
import role_assignment
import robocup
import planning_priority
import time
import skills.move


class Capture(single_robot_behavior.SingleRobotBehavior):
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

        self.add_transition(behavior.Behavior.State.start,
                            Capture.State.settle, lambda: True,
                            'immediately')

    def role_requirements(self):
        reqs = super().role_requirements()
        reqs.require_kicking = True
        # try to be near the ball
        if main.ball().valid:
            if main.ball().vel.mag() < self.InterceptVelocityThresh:
                reqs.destination_shape = main.ball().pos
            else:
                # TODO Make this less complicated and remove magic numbers
                reqs.destination_shape = robocup.Segment(main.ball().pos, main.ball().pos + main.ball().vel * 10)
        return reqs