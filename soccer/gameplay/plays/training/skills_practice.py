import robocup
import constants
import play
import enum
import behavior
import main


# Maintains the state of the ball's position by keeping track of which
# half the ball is on and prints on both entering a given state and
# continously during the execution of a given state.
class SkillsPractice(play.Play):
    class State(enum.Enum):
        # Define your states here.
        # eg: some_state = 0
        # -----------------------

    def __init__(self):
        super().__init__(continuous=True)

        # Register the states you defined using 'add_state'.
        # eg: self.add_state(WhichHalf.State.<???>,
        #                    behavior.Behavior.State.running)
        # ----------------------------------------------------

        self.add_transition(
            behavior.Behavior.State.start, behavior.Behavior.State.running, 
            lambda: True,
            "immediately")

        self.add_transition(
            behavior.Behavior.State.running, behavior.Behavior.State.completed
            lambda:self.subbehavior_with_name("skill").state == behavior.Behavior.State.completed,
            "done with behavior")

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

    def on_enter_running(self):
        # To make a robot move, use skills.move.Move(<point to move to>)
        # To create a point, we initialize a point using 
        # robocup.Point(<x coordinate>, <y coordinate>)
        
        # This line moves a robot to the point (0, 0)
        move_point = robocup.Point(0, 0)
        skill = skills.move.Move(move_point)

        # Adds behavior to our behavior tree, we will explain this more later
        self.add_subbehavior(move_skill, "skill", required=True)