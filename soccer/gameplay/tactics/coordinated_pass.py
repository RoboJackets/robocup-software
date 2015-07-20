import composite_behavior
import behavior
import skills.pivot_kick
import skills.pass_receive
import constants
import robocup
import time
import main
import enum
import logging


# This handles passing from one bot to another
# Simply run it and set it's receive point, the rest is handled for you
# It starts out by assigning a kicker and a receiver and instructing them to lineup for the pass
# Once they're aligned, the kicker kicks and the receiver adjusts itself based on the ball's movement
# Note: due to mechanical limitations, a kicker often gets stuck trying to adjust its angle while it's just outside of it's
#       aim error threshold.  If this happens, the CoordinatedPass will adjust the receive point slightly
#       because it's easier to move the receiver over a bit than have the kicker adjust its angle.  This solves
#       the problem of having a pass get stuck indefinitely while the kicker sits there not moving.
# TODO: # If an opponent blocks the pass channel, it will wait until it moves - you can cancel it at this point if you wish
# As soon as the kicker kicks, it is no longer and is released by this behavior so other behaviors can be assigned to it
# If the receiver gets the ball, CoordinatedPass transitions to the completed state, otherwise it goes to the failed state
class CoordinatedPass(composite_behavior.CompositeBehavior):

    KickPower = 0.6


    class State(enum.Enum):
        preparing = 1   # the kicker is aiming and the receiver is getting ready
        kicking = 2     # waiting for the kicker to kick
        receiving = 3   # the kicker has kicked and the receiver is trying to get the ball


    def __init__(self, receive_point=None):
        super().__init__(continuous=False)

        self.receive_point = receive_point

        for state in CoordinatedPass.State:
            self.add_state(state, behavior.Behavior.State.running)


        self.add_transition(behavior.Behavior.State.start,
            CoordinatedPass.State.preparing,
            lambda: True,
            'immediately')

        self.add_transition(CoordinatedPass.State.preparing,
            CoordinatedPass.State.kicking,
            lambda: (self.subbehavior_with_name('kicker').state == skills.pivot_kick.PivotKick.State.aimed
                and self.subbehavior_with_name('receiver').state == skills.pass_receive.PassReceive.State.aligned),
            'kicker and receiver ready')

        self.add_transition(CoordinatedPass.State.kicking,
            CoordinatedPass.State.receiving,
            lambda: self.subbehavior_with_name('kicker').state == behavior.Behavior.State.completed,
            'kicker kicked')

        self.add_transition(CoordinatedPass.State.receiving,
            behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name('receiver').state == behavior.Behavior.State.completed,
            'pass received!')

        self.add_transition(CoordinatedPass.State.receiving,
            behavior.Behavior.State.failed,
            lambda: self.subbehavior_with_name('receiver').state == behavior.Behavior.State.failed,
            'pass failed :(')


    # set the location where the receiving bot should camp out and wait for the ball
    # Default: None
    @property
    def receive_point(self):
        return self._receive_point
    @receive_point.setter
    def receive_point(self, value):
        self._receive_point = value

        # set receive_point for kicker and receiver (if present)
        if self.has_subbehavior_with_name('kicker'):
            self.subbehavior_with_name('kicker').target = self.receive_point
        if self.has_subbehavior_with_name('receiver'):
            self.subbehavior_with_name('receiver').receive_point = self.receive_point



    def on_enter_running(self):
        receiver = skills.pass_receive.PassReceive()
        receiver.receive_point = self.receive_point
        self.add_subbehavior(receiver, 'receiver', required=True)


    def on_exit_running(self):
        self.remove_subbehavior('receiver')


    def on_enter_kicking(self):
        self.subbehavior_with_name('kicker').enable_kick = True


    def on_enter_preparing(self):
        kicker = skills.pivot_kick.PivotKick()
        kicker.target = self.receive_point
        kicker.kick_power = CoordinatedPass.KickPower
        kicker.enable_kick = False # we'll re-enable kick once both bots are ready

        # we use tighter error thresholds because passing is hard
        kicker.aim_params['error_threshold'] = 0.2
        kicker.aim_params['max_steady_ang_vel'] = 3.0
        kicker.aim_params['min_steady_duration'] = 0.15
        kicker.aim_params['desperate_timeout'] = 3.0
        self.add_subbehavior(kicker, 'kicker', required=True)

        # receive point renegotiation
        self._last_unsteady_time = None
        self._has_renegotiated_receive_point = False


    def execute_running(self):
        # The shot obstacle doesn't apply to the receiver
        if self.has_subbehavior_with_name('kicker'):
            kicker = self.subbehavior_with_name('kicker')
            receiver = self.subbehavior_with_name('receiver')
            kicker.shot_obstacle_ignoring_robots = [receiver.robot]



    def execute_preparing(self):
        kicker = self.subbehavior_with_name('kicker')

        # receive point renegotiation
        # if the kicker sits there aiming close to target and gets stuck,
        # we set the receive point to the point the kicker is currently aiming at
        if kicker.current_shot_point() != None and not self._has_renegotiated_receive_point:
            if (not kicker.is_steady()
                and kicker.state == skills.pivot_kick.PivotKick.State.aiming):
                self._last_unsteady_time = time.time()

            if (self._last_unsteady_time != None
                and time.time() - self._last_unsteady_time > 0.75
                and kicker.current_shot_point().dist_to(self.receive_point) < 0.1):
                # renegotiate receive_point
                logging.info("Pass renegotiated RCV PT")
                self.receive_point = kicker.current_shot_point()
                self._has_renegotiated_receive_point = True


    def on_enter_receiving(self):
        # once the ball's been kicked, the kicker can go relax or do another job
        self.subbehavior_with_name('receiver').ball_kicked = True
        self.remove_subbehavior('kicker')

        



    def __str__(self):
        desc = super().__str__()
        desc += "\n    rcv_pt=" + str(self.receive_point)
        return desc
