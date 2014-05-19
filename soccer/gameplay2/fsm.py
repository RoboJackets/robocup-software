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


    def add_state(self, state, parent_state=None):
        if not isinstance(state, Enum):
            raise TypeError("State should be an Enum type")
        self._state_hierarchy[state] = parent_state


    def transition(self, new_state):
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
