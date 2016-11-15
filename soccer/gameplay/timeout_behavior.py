import composite_behavior
import behavior
import time
import enum


## This class adds a timeout to any subbehavior
#
# The way this works is by wrapping your desired behavior in this one.
# To use, simply add the behavior you want to timeout like this:
#
# self.add_subbihavior(timeout_behavior.TimeoutBehavior(instance_of_your_behavior). timeout),
# instead of normal timeouts
class TimeoutBehavior(composite_behavior.CompositeBehavior):
    class State(enum.Enum):
        timeout = 1

    ## Constructor for TimeoutBehavior
    #
    # @param subbehavior The subbehavior to wrap
    # @param timeout The timeout in seconds, to wait before killing this subbehavior
    def __init__(self, subbehavior, timeout):
        super().__init__(continuous=False)

        self.add_state(TimeoutBehavior.State.timeout,
                       behavior.Behavior.State.failed)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        self.add_transition(
            behavior.Behavior.State.running, TimeoutBehavior.State.timeout,
            lambda: self.timeout_exceeded(), 'Subbehavior timed out')

        self.add_transition(
            behavior.Behavior.State.running, behavior.Behavior.State.failed,
            lambda: self.subbehavior_with_name('toTimeout').state == behavior.Behavior.State.failed,
            'Subbehavior failed')

        self.add_transition(
            behavior.Behavior.State.running, behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name('toTimeout').state == behavior.Behavior.State.completed,
            'Subbehavior completed')

        self._behavior = subbehavior
        self.add_subbehavior(subbehavior, 'toTimeout')
        self._timeout = timeout
        self.start_time = time.time()

    @property

    ## Use this to access the wrapped behavior's variables and methods
    def behavior(self):
        return self._behavior

    @property
    def timeout(self):
        return self._timeout

    def timeout_exceeded(self):
        if self.time_elapsed() > self.timeout:
            return True
        return False

    def on_exit_running(self):
        self.remove_all_subbehaviors()

    ## Restarts the timer, which starts back at zero
    def restart_timer(self):
        self.start_time = time.time()

    def time_elapsed(self):
        return time.time() - self.start_time

    def time_remaining(self):
        return self.timeout - self.time_elapsed()

    ## Restarts this play, and restarts the timer
    #
    # To restart this play w/o restarting the timer, call the behavior's restart
    def restart(self):
        self.subbehavior_with_name('toTimeout').restart()
        super.restart()

    def __str__(self):
        desc = super().__str__()
        desc += "\n    time_remaining=" + str(round(self.time_remaining(),
                                                    2)) + "s"
        return desc
