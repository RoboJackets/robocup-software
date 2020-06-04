import composite_behavior
import behavior
import constants
import robocup
import time
import main
import enum
import logging
#add other required imports

## Broad, once sentence explanation of tactic
# More specific explanation, usually multiple lines
# ...
#


class NameOfTactic(
        composite_behavior.CompositeBehavior
):  #This creates the class of your tactic, and this is what's called when you want to use it in a play
    # Global variables in all caps SNAKE_CASE, ex:
    GLOBAL_VARAIBLE = 1

    class State(
            enum.Enum
    ):  #This creates the states that you transition between in your tactic
        #name in snake case
        state_one = 1  # short comment about what the state
        state_two = 2
        state_three = 3

    # This is what the class will do when it is called, the params list should always include self first
    def __init__(
        self,
        param_one=None,
        param_two=1,
        param_three=False
    ):  # These are the parameters your tactic takes in, they should be in snake case, and usually have default values
        super().__init__(
            continuous=False
        )  # Calls the constructor for composite behavior, should be set to continuous if tactic is going to run continuously

        # set parameters to be equal to self.the_param_name, so they can accessed in other funcitons in the file
        self.param_one = param_one
        self.param_two = param_two
        self.param_three = param_three

        #Other Variables can be added here (make sure to use self. ):
        self.msic_one = 1
        self.misc_two = False

        # For loop that basically makes it so that the state you made earlier are states that can be run
        for s in NameOfTactic.State:
            self.add_state(s, behavior.Behavior.State.running)

        # State transitions, here you specify when you are going to transition from one state to another
        # Note: Transitions do not have to be linear, but usually are for noncontinuous tactics

        # This tranistion should be in all tactics, it just goes from the starting state to your first state
        self.add_transition(behavior.Behavior.State.start,
                            NameOfTactic.State.state_one, lambda: True,
                            'immediately')

        self.add_transition(
            NameOfTactic.State.stae_one,  #from this state transition to:
            NameOfTactic.State.state_two,
            lambda: self.misc_one == 1 and self.func(),
            # Put in some conditional statement, can include functions
            'When 1 = 1'
        )  # The transition requires a name, usually a string describing in words when the transition happens

        # Do for rest of state transitions you want ...

        # Also add transitions for when the tactic completes and when it fails
        self.add_transition(NameOfTactic.State.state_three,
                            behavior.Behavior.State.completed,
                            lambda: self.misc_two, 'Tactic complete!')

        self.add_transition(NameOfTactic.State.state_three,
                            behavior.Behavior.State.failed,
                            lambda: not self.misc_two, 'Tactic failed :(')

    #Now we will define what happens in each state:
    def on_enter_state_one(self):
        # This function describes what will happen when we first enter state_one, this will only run once on entry of the state
        # Put code here ...

    def execute_state_one(self):
        # This function describes what will happen while we are in state_one, this will run continuously while in this state
        # Put code here ...

    def on_exit_state_one(self):
        # This function describes whatt will happen when transitioning out of the state, this will run once on exit of the state
        # Put code here ...
        # The on_exit usually also gets rid of any subbheviors that were running before
        # use


# This is a very basic template please look to other tactics for examples of what kind of things you can do