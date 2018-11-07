import robocup
import constants
import play
import enum
import behavior
import main


# Maintains the state of the ball's position by keeping track of which
# half the ball is on and prints on both entering a given state and
# continously during the execution of a given state.
class WhichHalf(play.Play):
    class State(enum.Enum):
        # Define your states here.
        # eg: some_state = 0
        # -----------------------
        righthalf = 0
        lefthalf = 1

    def __init__(self):
        super().__init__(continuous=True)

        # Register the states you defined using 'add_state'.
        # eg: self.add_state(WhichHalf.State.<???>,
        #                    behavior.Behavior.State.running)
        # ----------------------------------------------------

        # Assume we're either in tophalf or bottomhalf, no state for
        # being right on the line.
        self.add_state(WhichHalf.State.righthalf,
                       behavior.Behavior.State.running)
        self.add_state(WhichHalf.State.lefthalf,
                       behavior.Behavior.State.running)

        # Add your state transitions using 'add_transition'.
        # eg: self.add_transition(behavior.Behavior.State.start,
        #                         self.State.<???>, lambda: True,
        #                         'immediately')
        # eg: self.add_transition(self.State.<???>, self.State.<???>,
        #                         lambda: <???>,
        #                         'state change message')
        # ------------------------------------------------------------

        # Helps us be less redundant in the transition functions.
        in_right_half = (lambda: main.ball().pos.x <= 
                            0)

        # Rather than defining two more transitions from start to the top or
        # bottom half, we simply assume we start in bottom and let bottom's
        # transition function sort it out.
        self.add_transition(behavior.Behavior.State.start,
                            self.State.lefthalf, lambda: True, 'immediately')

        self.add_transition(self.State.lefthalf,
                            self.State.righthalf, lambda: not in_right_half(),
                            'detected left half')

        self.add_transition(self.State.righthalf, self.State.lefthalf,
                            in_right_half, 'detected right half')

    # Define your own 'on_enter' and 'execute' functions here.
    # eg: def on_enter_<???>(self):
    #         print('Something?')
    # eg: def execute_<???>(self):
    #         print('Something?')
    # ---------------------------------------------------------

    def on_enter_righthalf(self):
        print('Ball entered right half')

    def on_enter_lefthalf(self):
        print('Ball entered left half')

    def execute_righthalf(self):
        print('Ball in right half')

    def execute_lefthalf(self):
        print('Ball in left half')
