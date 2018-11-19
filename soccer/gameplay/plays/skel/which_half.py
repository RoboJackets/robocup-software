import robocup
import constants
import play
import enum
import behavior
import main

# Slides and other materials can be found here:
# https://github.com/RoboJackets/robocup-training
#
# Field Documentation can be found here:
# https://robojackets.github.io/robocup-software/struct_field___dimensions.html
#
# Ball Documentation can be found here:
# https://robojackets.github.io/robocup-software/class_ball.html


# Maintains the state of the ball's position by keeping track of which
# half the ball is on and prints on both entering a given state and
# continously during the execution of a given state.
class WhichHalf(play.Play):
    class State(enum.Enum):
        # Define your states here.
        # eg: staome_state = 0
        # -----------------------
        LeftHalf = 0
        RightHalf = 1
        pass  # remove this once you have put in your states

    def __init__(self):
        super().__init__(continuous=True)

        self.add_state(self.State.LeftHalf,
            behavior.Behavior.State.running)
        self.add_state(self.State.RightHalf,
            behavior.Behavior.State.running)
        
        self.add_transition(behavior.Behavior.State.start,
            self.State.RightHalf, lambda: True, 'Init')
        self.add_transition(self.State.RightHalf, 
            self.State.LeftHalf, in_left_half, 'Detected Left Half')
        self.add_transition(self.State.LeftHalf, 
            self.State.RightHalf, lambda : not in_left_half(), 'Deceted Right Half')


    in_left_half = (lambda x: main.ball().pos.x > constants.Field.Width / 2)

    def on_enter_LeftHalf(self):
        print('Ball entering Left Half')

    def on_exit_LeftHalf(self):
        print('Ball exiting Left Half')

    def execute_LeftHalf(self):
        print("Ball is in the Left Half")

    def on_enter_RightHalf(self):
        print('Ball is entering the right half')

    def on_exit_RightHalf(self):
        print('Ball is leaving the right half')

    def execute_RightHalf(self):
        print("Ball is chillin in the right half")

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

        # Define your own 'on_enter' and 'execute' functions here.
        # eg: def on_enter_<???>(self):
        #         print('Something?')
        # eg: def execute_<???>(self):
        #         print('Something?')
        # ---------------------------------------------------------
