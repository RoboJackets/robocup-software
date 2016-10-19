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

    # Setting up global class variables for location to put binaryclock.
    LeftPoint = -constants.Field.Width / 6
    RobotDist = (abs(LeftPoint) / 2)
    # Perfect alignment
    LeftPoint -= RobotDist / 2

    class State(enum.Enum):
        # We only need one state, and we'll transition to itself when we want to update.
        waiting = 0
        dummy = 1

    def __init__(self):
        super().__init__(continuous=True)

        # This is a local variable of this class
        # Refer to it with self.current_time
        self.current_time = self.get_time()

        # Register the states you defined using 'add_state'.
        # eg: self.add_state(WhichHalf.State.<???>,
        #                    behavior.Behavior.State.running)
        # ----------------------------------------------------
        self.add_state(BinaryClock.State.waiting,
                       behavior.Behavior.State.running)
        self.add_state(BinaryClock.State.dummy,
                       behavior.Behavior.State.running)

        # Add your state transitions using 'add_transition'.
        # eg: self.add_transition(behavior.Behavior.State.start,
        #                         self.State.<???>, lambda: True,
        #                         'immediately')
        # eg: self.add_transition(self.State.<???>, self.State.<???>,
        #                         lambda: <???>,
        #                         'state change message')
        # ------------------------------------------------------------

        # EXAMPLE TRANSITION, YOU MAY WANT TO REPLACE THIS
        self.add_transition(behavior.Behavior.State.start, self.State.waiting,
                            lambda: True, 'immediately')
        self.add_transition(self.State.waiting, self.State.dummy,
                            lambda: self.current_time != self.get_time(),
                            'Time in minutes changed')
        self.add_transition(self.State.dummy, self.State.waiting, lambda: True,
                            'immediately')

    # Define your own 'on_enter' and 'execute' functions here.
    # eg: def on_enter_<???>(self):
    #         print('Something?')
    # eg: def execute_<???>(self):
    #         print('Something?')
    # ---------------------------------------------------------

    def get_time(self):
        return time.localtime().tm_min

    # Demo of moving to a point.
    def on_enter_waiting(self):
        self.current_time = self.get_time()
        binary = format(self.current_time, '06b')
        move_point = robocup.Point(BinaryClock.LeftPoint,
                                   constants.Field.Length / 3)

        for i in range(6):
            if (binary[i] == '1'):
                self.add_subbehavior(
                    skills.move.Move(move_point), 'Robot' + str(i))
            move_point = robocup.Point(move_point.x + BinaryClock.RobotDist,
                                       move_point.y)

        # Swallow all unused robots
        self.add_subbehavior(plays.testing.line_up.LineUp(), 'line up')

    def on_exit_waiting(self):
        self.remove_all_subbehaviors()
