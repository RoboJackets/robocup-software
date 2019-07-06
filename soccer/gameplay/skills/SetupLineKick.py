import single_robot_composite_behavior
import behavior
import enum
import main
import role_assignment
import robocup
import constants
import math

# Captures the ball through whatever means needed
# In most cases, the robot moves in front of the moving ball
# to bounce it off the dribbler. It then slowly moves through the
# ball to fully control it
class SetupLineKick(single_robot_composite_behavior.SingleRobotCompositeBehavior):

    class State(enum.Enum):
        move = 1
        kick = 2

    def __init__(self):

        self.add_state(SetupLineKick.State.waiting, behavior.Behavior.State.running)
        self.add_state(SetupLineKick.State.kick, behavior.Behavior.State.running)

        self.enable_kick

        self.add_transition(behavior.Behavior.State.start,
                            SetupLineKick.State.waiting, lambda: True,
                            'immediately')
        self.add_transition(SetupLineKick.State.waiting,
                            LineKick.State.kick, lambda: self.enable_kick,
                            'kicker is enabled')
