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
        start = 0
        bottom = 1
        top = 2
        # Define your states here.
        # eg: some_state = 0
        # -----------------------
        pass  # remove this once you have put in your states

    def __init__(self):
        super().__init__(continuous=True)

        # Register the states you defined using 'add_state'.
        self.add_state(WhichHalf.State.bottom,
                       behavior.Behavior.State.running)
        self.add_state(WhichHalf.State.top,
                       behavior.Behavior.State.running)
        # ----------------------------------------------------

        # Add your state transitions using 'add_transition'.
        self.add_transition(behavior.Behavior.State.start,
                            self.State.bottom, lambda: True,
                            'immediately')
        self.add_transition(self.State.bottom, self.State.top,
                            lambda: main.ball().pos.y > constants.Field.length / 2,
                            'ball on top')
        self.add_transition(self.State.top, self.State.bottom,
                            lambda: main.ball().pos.y < constants.Field.length / 2,
                            'ball on bottom')
        # ------------------------------------------------------------

        # Define your own 'on_enter' and 'execute' functions here.
    def on_enter_top(self):
        print('The ball is on the top hald of the field.')  #no execcute method necessary because no execution --> just print statement
    def on_enter_bottom(self):
        print('The ball is on the bottom hald of the field.')
        # ---------------------------------------------------------
