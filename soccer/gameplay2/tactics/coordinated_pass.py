import composite_behavior
import behavior
import skills.pivot_kick
import skills.pass_receive
import enum
import robocup


# This handles passing from one bot to another
# Simply run it and set it's receive point, the rest is handled for you
# It starts out by assigning a kicker and a receiver and instructing them to lineup for the pass
# Once they're aligned, the kicker kicks and the receiver adjusts itself based on the ball's movement
# Note: due to mechanical limitations, a kicker often gets stuck trying to adjust its angle while it's just outside of it's
#       aim error threshold.  If this happens, the CoordinatedPass will adjust the receive point slightly
#       because it's easier to move the receiver over a bit than have the kicker adjust its angle.  This solves
#       the problem of having a pass get stuck indefinitely while the kicker sits there not moving.
# If an opponent blocks the pass channel, it will wait until it moves - you can cancel it at this point if you wish
# As soon as the kicker kicks, it is no longer and is released by this behavior so other behaviors can be assigned to it
# If the receiver gets the ball, CoordinatedPass transitions to the completed state, otherwise it goes to the failed state
class CoordinatedPass(composite_behavior.CompositeBehavior):

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

        self.add_transition(CoordinatedPass.State.kicking,
            CoordinatedPass.State.preparing,
            lambda: self.subbehavior_with_name('kicker').state == behavior.Behavior.State.aiming,
            'kicker misaligned')

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
        for bhvr in self.all_subbehaviors():
            bhvr.receive_point = value


    def on_enter_running(self):
        receiver = skills.pass_receive.PassReceive()
        receiver.receive_point = self.receive_point
        self.add_subbehavior(receiver, 'receiver', required=True)


    def on_exit_running(self):
        self.remove_subbehavior('receiver')


    def on_enter_preparing(self):
        kicker = skills.pivot_kick.PivotKick()
        kicker.receive_point = self.receive_point
        self.add_subbehavior(kicker, 'kicker', required=True)


    def on_enter_receiving(self):
        # once the ball's been kicked, the kicker can go relax or do another job
        self.remove_subbehavior('kicker')



    def __str__(self):
        desc = super().__str__()
        desc += "\n    rcv_pt=" + str(self.receive_point)
        return desc
