import robocup
import constants
import play
import enum
import behavior
import main
import skills.move
import plays.testing.line_up
import time


# Maintains the state of the ball's position by keeping track of which
# half the ball is on and prints on both entering a given state and
# continously during the execution of a given state.
class BinaryClock(play.Play):
    class State(enum.Enum):
        # Define your states here.
        # eg: some_state = 0
        # -----------------------
        pass  # remove this once you have put in your states

    def __init__(self):
        super().__init__(continuous=True)

        # This is a local variable of this class
        # Refer to it with self.current_time
        self.current_time = time.localtime().tm_min

        # Register the states you defined using 'add_state'.
        # eg: self.add_state(WhichHalf.State.<???>,
        #                    behavior.Behavior.State.running)
        # ----------------------------------------------------

        # Add your state transitions using 'add_transition'.
        # eg: self.add_transition(behavior.Behavior.State.start,
        #                         self.State.<???>, lambda: True,
        #                         'immediately')
        # eg: self.add_transition(self.State.<???>, self.State.<???>,
        #                         lambda: <???>,
        #                         'state change message')
        # ------------------------------------------------------------

        # EXAMPLE TRANSITION, YOU MAY WANT TO REPLACE THIS
        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

    # Define your own 'on_enter' and 'execute' functions here.
    # eg: def on_enter_<???>(self):
    #         print('Something?')
    # eg: def execute_<???>(self):
    #         print('Something?')
    # ---------------------------------------------------------

    # Demo of moving to a point.
    def on_enter_running(self):
        move_point = robocup.Point(0, constants.Field.Length / 2)
        self.add_subbehavior(skills.move.Move(move_point), 'test move')
