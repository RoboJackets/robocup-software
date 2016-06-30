import composite_behavior
import behavior
import time
import enum

class TimeoutBehavior(composite_behavior.CompositeBehavior):


    class State(enum.Enum):
        timeout = 1

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

        self._behavior = subbehavior
        self.add_subbehavior(subbehavior, 'toTimeout')
        self._timeout = timeout
        self.start_time = time.time()

    @property
    def behavior(self):
        return self._behavior

    @property
    def timeout(self):
        return self._timeout

    def timeout_exceeded(self):
        if time.time() - self.start_time > self.timeout:
            return True
        return False

    def on_enter_failed(self):
        self.remove_all_subbehaviors()

    def restart_timer(self):
        self.start_time = time.time()

    def restart(self):
        self.subbehavior_with_name('toTimeout').restart()
        super.restart()
