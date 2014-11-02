from enum import Enum
import fsm
import logging


# a Behavior is an abstract superclass for Skill, Play, etc
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
        super().__init__(start_state=Behavior.State.start)
        # add base states for Behavior
        self.add_state(Behavior.State.start)
        self.add_state(Behavior.State.running)
        self.add_state(Behavior.State.completed)
        self.add_state(Behavior.State.failed)
        self.add_state(Behavior.State.cancelled)

        self._is_continuous = continuous


    def add_state(self, state, parent_state=None):
        super().add_state(state, parent_state)
        #TODO: raise exception if @state doesn't have a Behavior.State ancestor


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
        state_desc = self.state.name if self.state != None else ""
        return self.__class__.__name__ + "::" + state_desc


    # Returns a tree of RoleRequirements keyed by subbehavior reference name
    # This is used by the dynamic role assignment system to
    # intelligently select which robot will run which behavior
    def role_requirements(self):
        raise NotImplementedError()


    # assignments is a tree of (RoleRequirements, OurRobot) tuples
    # Same tree structure as the role_requirements() return value, but tuples instead of RoleRequirements as leaf nodes
    def assign_roles(self, assignments):
        raise NotImplementedError()
