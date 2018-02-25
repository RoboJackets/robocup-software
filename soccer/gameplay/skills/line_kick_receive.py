import single_robot_composite_behavior
import behavior
import constants
import enum
import math
import time
import role_assignment
import skills
import main


## LineKickReceive accepts a receive_point as a parameter and gets setup there to catch the ball
# It transitions to the 'aligned' state once it's there within its error thresholds and is steady
# Set its 'ball_kicked' property to True to tell it to dynamically update its position based on where
# the ball is moving and attempt to catch it.
# It will move to the 'completed' state if it catches the ball, otherwise it will go to 'failed'.
class LineKickReceive(
        single_robot_composite_behavior.SingleRobotCompositeBehavior):

    ## how much we're allowed to be off in the direction of the pass line
    PositionErrorThreshold = 0.1

    class State(enum.Enum):
        ## we're aligning with the planned receive point
        aligning = 1

        ## being in this state signals that we're ready for the kicker to kick
        aligned = 2

        ## the ball's been kicked and we're adjusting based on where the ball's moving
        receiving = 3

    def __init__(self, captureFunction=(lambda: skills.capture.Capture())):
        super().__init__(continuous=False)

        self.ball_kicked = False
        self._receive_point = None
        self.target = constants.Field.TheirGoalSegment
        self.captureFunction = captureFunction

        for state in LineKickReceive.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            LineKickReceive.State.aligning, lambda: True,
                            'immediately')

        self.add_transition(LineKickReceive.State.aligning,
                            LineKickReceive.State.aligned, lambda: self.
                            errors_below_thresholds() and not self.ball_kicked,
                            'steady and in position to receive')

        self.add_transition(LineKickReceive.State.aligned,
                            LineKickReceive.State.aligning, lambda: not self.
                            errors_below_thresholds() and not self.ball_kicked,
                            'not in receive position')

        for state in [
                LineKickReceive.State.aligning, LineKickReceive.State.aligned
        ]:
            self.add_transition(
                state,
                LineKickReceive.State.receiving, lambda: self.ball_kicked,
                'ball kicked')

        self.add_transition(
            LineKickReceive.State.receiving,
            behavior.Behavior.State.completed, lambda: self.robot.has_ball(),
            'ball received!')

        # TODO add a failed state for this Receiver (possibly a timeout...)
        # self.add_transition(
        #    LineKickReceive.State.receiving, behavior.Behavior.State.failed,
        #    lambda: self.check_failure(), 'ball missed :(')

        ## set this to True to let the receiver know that the pass has started and the ball's in motion
        # Default: False
    @property
    def ball_kicked(self):
        return self._ball_kicked

    @ball_kicked.setter
    def ball_kicked(self, value):
        self._ball_kicked = value
        if value:
            self._ball_kick_time = time.time()

    ## The point that the receiver should expect the ball to hit it's mouth
    # Default: None
    @property
    def receive_point(self):
        return self._receive_point

    @receive_point.setter
    def receive_point(self, value):
        self._receive_point = value

    ## returns True if we're facing the right direction and in the right position and steady
    def errors_below_thresholds(self):
        if self.receive_point == None:
            return False

        return (self.robot.pos.dist_to(self.receive_point) <
                LineKickReceive.PositionErrorThreshold)

    def on_exit_start(self):
        # reset
        self.ball_kicked = False

    def execute_aligning(self):
        if self.receive_point != None:
            self.robot.move_to(self.receive_point)

    def on_enter_receiving(self):
        kick = skills.line_kick.LineKick()
        kick.shell_id = self.robot.shell_id()
        kick.target = self.target
        self.add_subbehavior(kick, 'kick', required=True)
        self.kicked_time = time.time()

    def execute_receiving(self):
        # Try to work around #755
        self.receive_point = main.ball().pos

    def on_exit_receiving(self):
        self.remove_subbehavior('kick')

    ## TODO ADD FAILURE CHECKER FOR FORWARD PASS RECEIVE
    # return currently returns false always
    def check_failure(self):
        return False

    ## prefer a robot that's already near the receive position
    def role_requirements(self):
        reqs = super().role_requirements()
        for req in role_assignment.iterate_role_requirements_tree_leaves(reqs):
            if self.receive_point != None:
                req.destination_shape = self.receive_point
            elif self.receive_point != None:
                req.destination_shape = self.receive_point
        return reqs

    def __str__(self):
        desc = super().__str__()
        if self.receive_point != None and self.robot != None:
            desc += "\n    receive_point=" + str(self.receive_point)
        return desc
