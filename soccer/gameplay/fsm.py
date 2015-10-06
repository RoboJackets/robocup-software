import logging
from enum import Enum
import graphviz as gv
import subprocess


## @brief generic hierarchial state machine class.
#
# states can have substates.  If the machine is in a state, then it is also implicitly in that state's parent state
# this basically provides for polymorphism/subclassing of state machines
#
# There are three methods corresponding to each state:
# * on_enter_STATE
# * execute_STATE
# * on_exit_STATE
#
# Subclasses of StateMachine can optionally implement them and they will automatically be called at the appropriate times.
class StateMachine:

    def __init__(self, start_state):
        # stores all states in the form _state_hierarchy[state] = parent_state
        self._state_hierarchy = {}
        self._transitions = {}
        self._start_state = start_state
        self._state = None


    @property
    def start_state(self):
        return self._start_state


    ## Resets the FSM back into the start state
    def restart(self):
        self.transition(self.start_state)


    ## Registers a new state (which can optionally be a substate of an existing state)
    def add_state(self, state, parent_state=None):
        if not isinstance(state, Enum):
            raise TypeError("State should be an Enum type")
        self._state_hierarchy[state] = parent_state


    ## Runs the FSM
    # checks transition conditions for all edges leading away from the current state
    # if one evaluates to true, we transition to it
    # if more than one evaluates to true, we throw a RuntimeError
    def spin(self):
        s1 = self.state

        # call execute_STATENAME
        if self.state != None:
            for state in self.ancestors_of_state(self.state) + [self.state]:
                method_name = "execute_" + state.name
                state_method = None
                try:
                    state_method = getattr(self, method_name)
                except AttributeError:
                    pass
                if state_method is not None:
                    state_method()

        if self.state == None:
            self.transition(self.start_state)
        else:
            # transition if an 'event' fires
            next_states = []
            if self.state in self._transitions:
                for next_state, transition in self._transitions[self.state].items():
                    if transition['condition']():
                        next_states += [next_state]

            if len(next_states) > 1:
                logging.warn("Ambiguous fsm transitions from state'" + str(self.state) + "'.  The following states are reachable now: " + str(next_states) + ";  Proceeding by taking the first option.")
            if len(next_states) > 0:
                self.transition(next_states[0])

        # if a transition occurred during the spin, we'll spin again
        # note: this could potentially cause infinite recursion (although it shouldn't)
        if s1 != self.state:
            StateMachine.spin(self)




    # if you add a transition that already exists, the old one will be overwritten
    def add_transition(self, from_state, to_state, condition, event_name):
        if from_state not in self._transitions:
            self._transitions[from_state] = {}

        self._transitions[from_state][to_state] = {'condition': condition, 'name': event_name}


    # sets @state to the new_state given
    # calls 'on_exit_STATENAME()' if it exists
    # calls 'on_enter_STATENAME()' if it exists
    def transition(self, new_state):
        # print("TRANSITION: " + str(self.__class__.__name__) + ": " + str(self.state) + " -> " + str(new_state))
        if self.state != None:
            for state in self.ancestors_of_state(self.state) + [self.state]:
                if not self.state_is_substate(new_state, state):
                    method_name = "on_exit_" + state.name
                    state_method = None
                    try:
                        state_method = getattr(self, method_name)    # call the transition FROM method if it exists
                    except AttributeError:
                        pass
                    if state_method is not None:
                        state_method()

        for state in self.ancestors_of_state(new_state) + [new_state]:
            if not self.state_is_substate(self.state, state):
                method_name = "on_enter_" + state.name
                state_method = None
                try:
                    state_method = getattr(self, method_name)    # call the transition TO method if it exists
                except AttributeError:
                    pass
                if state_method is not None:
                    state_method()

        self._state = new_state


    # traverses the state hierarchy to see if it's in @state or one of @state's descendent states
    def is_in_state(self, state):
        return self.state_is_substate(self.state, state)


    def state_is_substate(self, state, possible_parent):
        ancestor = state
        while ancestor != None:
            if possible_parent == ancestor: return True
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


    # returns a list of the ancestors of the given state
    # if B is a child state of A and C is a child state of B, ancestors_of_state(C) == [A, B]
    # if @state has no ancestors, returns an empty list
    def ancestors_of_state(self, state):
        ancestors = []
        state = self._state_hierarchy[state]
        while state != None:
            ancestors.insert(0, state)
            state = self._state_hierarchy[state]
        return ancestors


    # returns a graphviz.Digraph object
    def as_graphviz(self):
        g = gv.Digraph(self.__class__.__name__)

        cluster_index = 0
        subgraphs = {}
        subgraphs[None] = g
        for state in self._state_hierarchy:
            if state not in subgraphs and state in self._state_hierarchy.values():
                sg = gv.Digraph('cluster_' + str(cluster_index), graph_attr={'label': state.__module__ + "::" + state.name, 'style': 'dotted'})
                cluster_index += 1

                subgraphs[state] = sg

        for state in self._state_hierarchy:
            has_children = state in self._state_hierarchy.values()

            if not has_children:
                enclosing_graph = subgraphs[self._state_hierarchy[state]]
                shape = 'diamond' if state == self.start_state else 'ellipse'
                enclosing_graph.node(state.name, label=state.__module__ + "::" + state.name, shape=shape)

        for state, subgraph in subgraphs.items():
            if state != None:
                subgraphs[self._state_hierarchy[state]].subgraph(subgraph)

        for start in self._transitions:
            for end, event in self._transitions[start].items():
                g.edge(start.name, end.name, label=event['name'], decorate='True')

        return g


    # returns a buffer containing the source code needed to generate a representative graphviz graph
    def to_graphviz(self):
        return self.as_graphviz().source.encode('latin-1')


    # writes a png file of the graphviz output to the specified location
    def write_diagram_png(self, filename):
        p = subprocess.Popen(['dot', '-Tpng', '-o' + filename], stdin=subprocess.PIPE)
        p.communicate(input=self.to_graphviz())


    @property
    def state(self):
        return self._state
