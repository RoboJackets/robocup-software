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
        tophalf = 0
        bottomhalf = 1

    def __init__(self):
        super().__init__(continuous=True)

        # Register the states you defined using 'add_state'.
        # eg: self.add_state(WhichHalf.State.<???>,
        #                    behavior.Behavior.State.running)
        # ----------------------------------------------------

        # Assume we're either in tophalf or bottomhalf, no state for
        # being right on the line.
        self.add_state(WhichHalf.State.tophalf,
                       behavior.Behavior.State.running)
        self.add_state(WhichHalf.State.bottomhalf,
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
        in_bottom_half = (
            lambda: main.ball().pos.y <= constants.Field.Length / 2)

        # Rather than defining two more transitions from start to the top or
        # bottom half, we simply assume we start in bottom and let bottom's
        # transition function sort it out.
        self.add_transition(behavior.Behavior.State.start,
                            self.State.bottomhalf, lambda: True, 'immediately')

        self.add_transition(self.State.bottomhalf, self.State.tophalf,
                            lambda: not in_bottom_half(), 'detected top half')

        self.add_transition(self.State.tophalf, self.State.bottomhalf,
                            in_bottom_half, 'detected bottom half')

    # Define your own 'on_enter' and 'execute' functions here.
    # eg: def on_enter_<???>(self):
    #         print('Something?')
    # eg: def execute_<???>(self):
    #         print('Something?')
    # ---------------------------------------------------------

    def on_enter_tophalf(self):
        print('Ball entered top half')

    def on_enter_bottomhalf(self):
        print('Ball entered bottom half')

    def execute_tophalf(self):
        print('Ball in top half')

    def execute_bottomhalf(self):
        print('Ball in bottom half')
