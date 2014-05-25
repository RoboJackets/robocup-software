from enum import Enum
import fsm

# role allocation parameters?  return a MotionConstraints object?  accept a MotionConstraints object?
# FIXME: what if it's done running, but still pending success evaluation??
# FIXME: parent and child behaviors?


# a Behavior is an abstract superclass for skill, play, etc
class Behavior(fsm.StateMachine):

    # These are the core states of the Behavior class
    # Subclasses may extend this by adding substates of the following
    class State(Enum):
        start = 1
        running = 2
        completed = 3

        # note: these two states are only relevant for non-continuous behaviors
        failed = 4
        cancelled = 5


    def __init__(self, continuous):
        super().__init__()
        # add base states for Behavior
        self.add_state(Behavior.State.start)
        self.add_state(Behavior.State.running)
        self.add_state(Behavior.State.completed)
        if not continuous:
            self.add_state(Behavior.State.failed)
            self.add_state(Behavior.State.cancelled)

        self._state = Behavior.State.start;
        self._is_continuous = continuous


    def add_state(self, state, parent_state=None):
        super().add_state(state, parent_state)
        #TODO: raise exception if @state doesn't have a Behavior.State ancestor


    # default implementation of the start state just takes us to the running state
    def execute_start(self):
        pass

    def execute_running(self):
        pass

    def execute_failed(self):
        pass

    def execute_completed(self):
        pass
    
    def execute_cancelled(self):
        pass


    def is_done_running(self):
        for state in [Behavior.State.completed, Behavior.State.failed, Behavior.State.cancelled]:
            if self.is_in_state(state): return True

        return False


    def terminate(self):
        if self.is_done_running():
            logging.warn("Attempt to terminate behavior that's already done running")
        else:
            if self.is_continuous:
                self.transition(Behavior.State.completed)
            else:
                self.transition(Behavior.State.cancelled)


    # returns a state in Behavior.State that represents what the behaviors is doing
    # use this instead of the @state property if you don't want to avoid dealing with custom subclass substates
    @property
    def behavior_state(self):
        return self.corresponding_ancestor_state(list(Behavior.State))


    # noncontinuous: a behavior that accomplishes a specific task, then completes (example: shooting at the goal)
    # continuous: a behavior that continually runs until told to stop (example: zone defense)
    @property
    def is_continuous(self):
        return self._is_continuous


    def __str__(self):
        return self.__class__.__name__ + "::" + self.state.name
