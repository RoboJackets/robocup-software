import single_robot_behavior
import behavior
import enum


# The Aim skill is used when a robot has the ball to aim at a particular target
# It fails if the ball is fumbled
# This behavior is continuous, meaning that once the aim is 'good', it continues running
# rather than entering the 'completed' state.  To indicate that the aim is good, it enters
# the 'aimed' state, which is a substate of running.  It may change back from 'aimed' to
# 'aiming' if it's parameters change or due to external conditions.
class Aim(single_robot_behavior.SingleRobotBehavior):

    class State(enum.Enum):
        aiming = 1
        aimed = 2


    def __init__(self):
        super().__init__(continuous=True)

        self.add_state(Aim.State.aiming, behavior.Behavior.State.running)
        self.add_state(Aim.State.aimed, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            Aim.State.aiming,
            lambda: True,
            'immediately')

        for state in Aim.State:
            self.add_transition(state,
                behavior.Behavior.State.failed,
                lambda: not self.robot.has_ball(),
                'fumble')

        self.add_transition(Aim.State.aiming,
            Aim.State.aimed,
            lambda: self.is_aimed(),
            'error < threshold and not rotating too fast')

        self.add_transition(Aim.State.aimed,
            Aim.State.aiming,
            lambda: not self.is_aimed(),
            'error > threshold or rotating too fast')


    # returns True if we're aimed at our target within our error thresholds and we're not rotating too fast
    def is_aimed(self):
        raise NotImplementedError()


    # we're aiming at a particular point on our target segment, what is this target?
    def aim_target_point(self):
        raise NotImplementedError()


    def execute_aiming(self):
        # TODO: port the functionality from PivotKick
        raise NotImplementedError()


    def execute_aimed(self):
        self.robot.face(self.aim_target_point())

