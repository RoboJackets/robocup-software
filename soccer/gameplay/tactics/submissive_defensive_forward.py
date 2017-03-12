import main
import robocup
import behaviors
import constants
import enum
import single_robot_behavior
import skills.mark

# Alternates between marking and collecting the ball
class SubmissiveDefensiveForward(single_robot_behavior.SingleRobotBehavior):
    class State(enum.Enum):
        # Block shots/passes
        blocking = 1
        # Collect the ball when it is lost
        collecting = 2

    def __init__(self):
        super().__init__(continuous=True)

        self.mark_robot = None
        self.mark_pos = None

        for s in SubmissiveDefensiveForward.State:
            self.add_state(s, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            SubmissiveDefensiveForward.State.blocking,
                            lambda: True,
                            'immediately')

        self.add_transition(SubmissiveDefensiveForward.State.blocking,
                            SubmissiveDefensiveForward.State.collecting,
                            lambda: within_range(),
                            'Collect Ball')

        self.add_transition(SubmissiveDefensiveForward.State.collecting,
                            SubmissiveDefensiveForward.State.blocking,
                            lambda: not within_range(),
                            'Block')

    # Wether we can collect the ball before the opponent
    def within_range():
        # Switch when fast estimated distance is < closest opp fast estimated distance
        # Fast estimated distance is line shifted X amnt in opposite direction of opponent robots
        # Do this for oppents
        # Check who gets there first
        # Percent error most likely
        # Leave good amount to play with
        return False

    # Create mark
    def on_enter_blocking(self):
        pass

    # Update mark based on parent
    def execute_blocking(self):
        pass

    # Clear everything
    def on_exit_blocking(self):
        self.remove_all_subbehaviors()

    # Start collection routine
    def on_enter_collecting(self):
        pass

    def on_exit_collecting(self):
        self.remove_all_subbehaviors()


    # Robot to mark
    @property
    def mark_robot(self):
        return self.mark_robot

    @mark_robot.setter
    def mark_robot(self, value):
        self.mark_robot = value;

    # Position to mark
    @property
    def mark_pos(self):
        return self.mark_pos

    @mark_pos.setter
    def mark_pos(self, value):
        self.mark_pos = value;