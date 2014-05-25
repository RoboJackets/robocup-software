import logging
from enum import Enum


# generic hierarchial state machine class
# states can have substates.  If the machine is in a state, then it is also implicitly in that state's parent state
# this basically provides for polymorphism/subclassing of state machines
class StateMachine:

    def __init__(self):
        # stores all states in the form _state_hierarchy[state] = parent_state
        self._state_hierarchy = {}
        self._state = None
        self._transitions = {}


    def add_state(self, state, parent_state=None):
        if not isinstance(state, Enum):
            raise TypeError("State should be an Enum type")
        self._state_hierarchy[state] = parent_state


    # checks transition conditions for all edges leading away from the current state
    # if one evaluates to true, we transition to it
    # if more than one evaluates to true, we throw a RuntimeError
    def run(self):
        # call execute_STATENAME
        # FIXME: if a transition occurs during run(), we should call the new
        # state's execute method so there's not a "propogation delay" for state transitions
        if self.state != None:
            method_name = "execute_" + self.state.name
            state_method = None
            try:
                state_method = getattr(self, method_name)
            except AttributeError:
                raise NotImplementedError("Fsm '" + self.__class__.__name__ + "' needs to implement '" + method_name + "()'")

            state_method()

        # transition if an 'event' fires
        next_states = []
        for next_state, condition in self._transitions[self.state].items():
            if condition():
                next_states += [next_state]

        if len(next_states) > 1:
            raise RuntimeError("Ambiguous fsm transitions from state'" + str(self.state) + "'.  The following states are reachable now: " + str(next_states))
        elif len(next_states) == 1:
            self.transition(next_states[0])


    # if you add a transition that already exists, the old one will be overwritten
    def add_transition(self, from_state, to_state, condition):
        if from_state not in self._transitions:
            self._transitions[from_state] = {}

        self._transitions[from_state][to_state] = condition


    # sets @state to the new_state given
    # calls 'on_enter_STATENAME()' if it exists
    def transition(self, new_state):
        method_name = "on_enter_" + new_state.name
        try:
            getattr(self, method_name)()    # call the transition TO method if it exists
        except AttributeError:
            pass

        self._state = new_state


    # traverses the state hierarchy to see if it's in @state or one of @state's descendent states
    def is_in_state(self, state):
        ancestor = self.state
        while ancestor != None:
            if state == ancestor: return True
            ancestor = self._state_hierarchy[ancestor]

        return False


    # looks at the list @ancestors and returns the one that the current state is a descendant of
    # returns None if the current state doesn't descend from one in the list
    def corresponding_ancestor_state(self, ancestors):
        state = self.state
        while state != None:
            if state in ancestors:
                return state
            state = self._state_hierarchy[state]

        return None


    @property
    def state(self):
        return self._state
