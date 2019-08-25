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


## This handles passing from one bot to another
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
        preparing = 1  # the kicker is aiming and the receiver is getting ready
        kicking = 2  # waiting for the kicker to kick
        receiving = 3  # the kicker has kicked and the receiver is trying to get the ball
        timeout = 4

    ## Init method for CoordinatedPass
    # @param skillreceiver an instance of a class that will handle the receiving robot. See pass_receive and angle_receive for examples.
    # Using this, you can change what the receiving robot does (rather than just receiving the ball, it can pass or shoot it).
    # Subclasses of pass_receive are preferred, but check the usage of this variable to be sure.
    # @param receive_point The point that will be kicked too. (Target point)
    # @param skillkicker A tuple of this form (kicking_class instance, ready_lambda). If none, it will use (pivot_kick lambda x: x == pivot_kick.State.aimed).
    # @param receiver_required Whether the receiver subbehavior should be required or not
    # @param kicker_required Whether the kicker subbehavior should be required or not
    # The lambda equation is called (passed with the state of your class instance) to see if your class is ready. Simple implementations will just compare it to your ready state.
    def __init__(self,
                 receive_point=None,
                 skillreceiver=None,
                 skillkicker=None,
                 prekick_timeout=None,
                 receiver_required=True,
                 kicker_required=True,
                 use_chipper=False):
        super().__init__(continuous=False)

        # This creates a new instance of skillreceiver every time the constructor is
        # called (instead of pulling from a single static instance).
        if skillreceiver == None:
            skillreceiver = skills.pass_receive.PassReceive()

        if skillkicker == None:
            skillkicker = (skills.pivot_kick.PivotKick(), lambda x: x ==
                           skills.pivot_kick.PivotKick.State.aimed)

        self.receive_point = receive_point
        self.skillreceiver = skillreceiver
        self.skillkicker = skillkicker
        self.skillkicker[0].use_chipper = use_chipper
        self.prekick_timeout = prekick_timeout
        self.receiver_required = receiver_required
        self.kicker_required = kicker_required

        self.add_state(CoordinatedPass.State.preparing,
                       behavior.Behavior.State.running)
        self.add_state(CoordinatedPass.State.kicking,
                       behavior.Behavior.State.running)
        self.add_state(CoordinatedPass.State.receiving,
                       behavior.Behavior.State.running)
        self.add_state(CoordinatedPass.State.timeout,
                       behavior.Behavior.State.failed)

        self.add_transition(behavior.Behavior.State.start,
                            CoordinatedPass.State.preparing, lambda: True,
                            'immediately')

        self.add_transition(
            CoordinatedPass.State.preparing, CoordinatedPass.State.kicking,
            lambda: (skillkicker[1](self.subbehavior_with_name('kicker').state)
                     and self.subbehavior_with_name('receiver').state == self.
                     skillreceiver.State.aligned), 'kicker and receiver ready')

        self.add_transition(
            CoordinatedPass.State.preparing, CoordinatedPass.State.timeout,
            self.prekick_timeout_exceeded, 'Timed out on prepare')

        self.add_transition(
            CoordinatedPass.State.kicking, CoordinatedPass.State.timeout,
            self.prekick_timeout_exceeded, 'Timed out on prepare')

        self.add_transition(CoordinatedPass.State.kicking,
                            CoordinatedPass.State.timeout,
                            self.prekick_timeout_exceeded, 'Timed out on kick')

        self.add_transition(CoordinatedPass.State.kicking,
                            CoordinatedPass.State.receiving, lambda: self.
                            subbehavior_with_name('kicker').state == behavior.
                            Behavior.State.completed, 'kicker kicked')

        self.add_transition(
            CoordinatedPass.State.receiving, behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name('receiver').state == behavior.
            Behavior.State.completed, 'pass received!')

        self.add_transition(
            CoordinatedPass.State.receiving,
            behavior.Behavior.State.failed, lambda: self.subbehavior_with_name(
                'receiver').state == behavior.Behavior.State.failed,
            'pass failed :(')

    ## Handles restarting this behaivor.
    # Since we save a few sub-behaviors, we need to restart those when we restart.
    def restart(self):
        super().restart()
        self.skillreceiver.restart()
        self.skillkicker[0].restart()

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
            self.subbehavior_with_name(
                'receiver').receive_point = self.receive_point

    def on_enter_running(self):
        kicker = self.skillkicker[0]
        kicker.target = self.receive_point
        kickpower = (main.ball().pos - self.receive_point).mag() / 17

        kickpower = max(0.05, min(kickpower, 1.0))

        # Very simple tuning right now
        # It is setup to use kickerpower once the distance scale has been
        # tuned correctly
        kicker.kick_power = 0.6  #kickpower
        kicker.enable_kick = False  # we'll re-enable kick once both bots are ready

        # we use tighter error thresholds because passing is hard
        kicker.aim_params['error_threshold'] = 0.1
        kicker.aim_params['max_steady_ang_vel'] = 0.1
        kicker.aim_params['min_steady_duration'] = 0.15
        kicker.aim_params['desperate_timeout'] = 2.0
        self.add_subbehavior(
            kicker, 'kicker', required=self.kicker_required, priority=5)
        receiver = self.skillreceiver
        receiver.receive_point = self.receive_point
        self.add_subbehavior(
            receiver, 'receiver', required=self.receiver_required, priority=5)

    @property
    def use_chipper(self):
        return self.skillkicker[0].use_chipper

    @use_chipper.setter
    def use_chipper(self, value):
        if isinstance(value, bool):
            self.skillkicker[0].use_chipper = value

    def on_exit_running(self):
        self.remove_subbehavior('receiver')

    def on_enter_kicking(self):
        self.subbehavior_with_name('kicker').enable_kick = True

    def on_enter_preparing(self):

        # receive point renegotiation
        self._last_unsteady_time = None
        self._has_renegotiated_receive_point = False

        self._preparing_start = time.time()

    def execute_running(self):
        # The shot obstacle doesn't apply to the receiver
        if self.has_subbehavior_with_name('kicker'):
            kicker = self.subbehavior_with_name('kicker')
            receiver = self.subbehavior_with_name('receiver')
            kicker.shot_obstacle_ignoring_robots = [receiver.robot]

    # gets robots involved with the pass
    def get_robots(self):
        kicker = None
        receiver = None
        if self.has_subbehavior_with_name('kicker'):
            kicker = self.subbehavior_with_name('kicker')
        if self.has_subbehavior_with_name('receiver'):
            receiver = self.subbehavior_with_name('receiver')
        toReturn = []
        if receiver is not None and receiver.robot is not None:
            toReturn.extend([receiver.robot])
        if kicker is not None and kicker.robot is not None:
            toReturn.extend([kicker.robot])
        return toReturn

    def execute_preparing(self):
        kicker = self.subbehavior_with_name('kicker')

        # receive point renegotiation
        # if the kicker sits there aiming close to target and gets stuck,
        # we set the receive point to the point the kicker is currently aiming at
        if kicker.current_shot_point(
        ) != None and not self._has_renegotiated_receive_point:
            if (not kicker.is_steady() and self.skillkicker[1](kicker.state)):
                self._last_unsteady_time = time.time()

            if (self._last_unsteady_time != None and
                    time.time() - self._last_unsteady_time > 0.75 and
                    kicker.current_shot_point().dist_to(self.receive_point) <
                    0.1):
                # renegotiate receive_point
                logging.info("Pass renegotiated RCV PT")
                self.receive_point = kicker.current_shot_point()
                self._has_renegotiated_receive_point = True

    def prekick_timeout_exceeded(self):
        if self._preparing_start == None or self.prekick_timeout == None or self.prekick_timeout <= 0:
            return False
        if time.time() - self._preparing_start > self.prekick_timeout:
            return True
        return False

    def time_remaining(self):
        if self._preparing_start == None or self.prekick_timeout == None or self.prekick_timeout <= 0:
            return 0
        return self.prekick_timeout - (time.time() - self._preparing_start)

    def on_enter_receiving(self):
        # once the ball's been kicked, the kicker can go relax or do another job
        self.subbehavior_with_name('receiver').ball_kicked = True
        self.remove_subbehavior('kicker')

    @property
    def has_roles_assigned(self):
        if self.state == CoordinatedPass.State.kicking:
            return len(self.get_robots()) == 2
        elif self.state == CoordinatedPass.State.receiving:
            return len(self.get_robots()) >= 1
        elif self.state == CoordinatedPass.State.preparing:
            return True
        else:
            return False

    def __str__(self):
        desc = super().__str__()
        desc += "\n    rcv_pt=" + str(self.receive_point)
        if not (self._preparing_start == None or self.prekick_timeout == None
                or self.prekick_timeout <= 0):
            desc += "\n    timeout=" + str(round(self.time_remaining(), 2))
        return desc
