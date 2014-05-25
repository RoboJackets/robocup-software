from behavior import Behavior
import time
from enum import Enum


# allows us to easily put a time-limit on the execution of a behavior
class TimedBehavior(Behavior):

    # having a separate timed_out state rather than just using the "failed" state from Behavior
    # allows us to differentiate between timing out and the enclosed behavior failing
    class State(Enum):
        timed_out = 1


    def __init__(self, behavior, time_limit):
        super().__init__(continuous=False)

        if not isinstance(behavior, Behavior):
            raise TypeError("Must give TimedBehavior a Behavior to time")

        self.add_state(TimedBehavior.State.timed_out, Behavior.State.failed)

        self._behavior = behavior
        self._time_limit = time_limit
        self._start_time = None


    def execute_start(self):
        self._start_time = time.time()
        self.transition(Behavior.State.running)


    def execute_running(self):
        if self.behavior.is_done_running():
            self.transition(self.behavior.behavior_state)
        else:
            if time.time() - self.start_time > self.time_limit:
                self.transition(TimedBehavior.State.timed_out)
                self.behavior.terminate()
            else:
                self.behavior.run()


    def terminate(self):
        self.behavior.terminate()
        super().terminate()


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

