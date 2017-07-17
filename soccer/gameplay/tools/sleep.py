import composite_behavior
import behavior
import time

## A behavior that simply waits for a specified time, in seconds, and completes once it is reached
# Very useful with behavior sequences
# also see timeout_behavior.py, for a behavior designed to timeout other behaviors


class SleepBehavior(composite_behavior.CompositeBehavior):
    def __init__(self, sleep_time):
        super().__init__(continuous=False)
        self.sleep_time = sleep_time

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        self.start_time = 0
        self.add_transition(
            behavior.Behavior.State.running, behavior.Behavior.State.completed,
            lambda: time.time() - self.start_time > self.sleep_time,
            'Waking up!')

    def on_enter_running(self):
        self.start_time = time.time()
