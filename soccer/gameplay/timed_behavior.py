from behavior import *
import time
from enum import Enum


## TimedBehavior wraps a subbehavior and allows us to easily place a time limit on it.
# After the specified amount of time has elapsed, the wrapped behavior is cancelled.
class TimedBehavior(Behavior):


    class State(Enum):

        ## Having a separate timed_out state rather than just using the "failed" state from Behavior
        # allows us to differentiate between timing out and the enclosed behavior failing
        timed_out = 1


    def __init__(self, behavior, time_limit):
        super().__init__(continuous=False)

        if not isinstance(behavior, Behavior):
            raise TypeError("Must give TimedBehavior a Behavior to time")

        self.add_state(TimedBehavior.State.timed_out, Behavior.State.failed)

        self._behavior = behavior
        self._time_limit = time_limit
        self._start_time = None

        self.add_transition(Behavior.State.start, Behavior.State.running, lambda: True, 'immediately')
        self.add_transition(Behavior.State.running, TimedBehavior.State.timed_out,
            lambda:
                time.time() - self.start_time > self.time_limit,
            'time runs out'
            )
        self.add_transition(Behavior.State.running, Behavior.State.completed,
            lambda:
                self.behavior.is_in_state(Behavior.State.completed),
            'subbehavior completed'
            )


    def on_enter_timed_out(self):
        self.behavior.terminate()


    def execute_start(self):
        self._start_time = time.time()


    def execute_running(self):
        self.behavior.run()


    def on_enter_cancelled(self):
        if not self.behavior.is_done_running():
            self.behavior.terminate()


    @property
    def start_time(self):
        return self._start_time


    @property
    def time_limit(self):
        return self._time_limit


    @property
    def behavior(self):
        return self._behavior


    def __str__(self):
        return super().__str__() + "\n\t" + str(self.behavior)

