import logging
from enum import Enum
from typing import (
    Union,
    Callable,
    Dict,
    Optional,
    TypedDict,
)

State = Enum
TransitionFunction = Callable[[], bool]  # Takes no args, returns a bool


class Event(TypedDict):
    condition: TransitionFunction
    name: str


TransitionTable = Dict[State, Dict[State, Event]]  # [from][to] = Event
StateMethod = Callable[[], None]  # Takes nothing, returns nothing
OnEnterMethod = Callable[[], None]  # Takes nothing, returns nothing
OnExitMethod = Callable[[], None]  # Takes nothing, returns nothing


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
    def __init__(self, start_state: State):
        self._transitions: TransitionTable = {}
        self._start_state: State = start_state
        self._state: Optional[State] = None

    @property
    def start_state(self) -> State:
        return self._start_state

    ## Resets the FSM back into the start state
    def restart(self) -> None:
        self.transition(self.start_state)

    ## Runs the FSM
    # checks transition conditions for all edges leading away from the current state
    # if one evaluates to true, we transition to it
    # if more than one evaluates to true, we throw a RuntimeError
    def tick(self) -> None:
        # call execute_STATENAME
        if self.state is not None:
            method_name = "execute_" + self.state.name
            exec_callback: Optional[StateMethod] = None
            try:
                exec_callback = getattr(self, method_name)
            except AttributeError:
                pass
            if exec_callback is not None:
                exec_callback()

        if self.state is None:
            self.transition(self.start_state)
        else:
            # transition if an 'event' fires
            next_states = []
            if self.state in self._transitions:
                for next_state, transition in self._transitions[self.state].items():
                    if transition["condition"]():
                        next_states += [next_state]

            if len(next_states) > 1:
                logging.warning(
                    "Ambiguous fsm transitions from state'"
                    + str(self.state)
                    + "'.  The following states are reachable now: "
                    + str(next_states)
                    + ";  Proceeding by taking the first option."
                )
            if len(next_states) > 0:
                self.transition(next_states[0])

    # if you add a transition that already exists, the old one will be overwritten
    def add_transition(
        self,
        from_state: State,
        to_state: State,
        condition: Union[bool, TransitionFunction],
        event_name: str,
    ) -> None:
        if isinstance(condition, bool):
            condition_fn = lambda: condition
        else:
            condition_fn = condition

        if from_state not in self._transitions:
            self._transitions[from_state] = {}

        self._transitions[from_state][to_state] = {
            "condition": condition_fn,
            "name": event_name,
        }

    # sets @state to the new_state given
    # calls 'on_exit_STATENAME()' if it exists
    # calls 'on_enter_STATENAME()' if it exists
    def transition(self, new_state: State) -> None:
        if self.state is not None:
            method_name = "on_exit_" + self.state.name
            exit_callback: Optional[OnExitMethod] = None
            try:
                exit_callback = getattr(
                    self, method_name
                )  # call the transition FROM method if it exists
            except AttributeError:
                pass
            if exit_callback is not None:
                exit_callback()

            method_name = "on_enter_" + new_state.name  # pylint: disable=no-member
            enter_callback: Optional[OnEnterMethod] = None
            try:
                enter_callback = getattr(
                    self, method_name
                )  # call the transition TO method if it exists
            except AttributeError:
                pass
            if enter_callback is not None:
                enter_callback()

        self._state = new_state

    @property
    def state(self) -> Optional[State]:
        return self._state
