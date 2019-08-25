import play
import behavior
import tactics.line_up
import robocup
import constants
import enum
import time


## Robots repeatedly line up on opposite sides of the field
class RepeatedTurningLineUpLengthwise(play.Play):

    Pause = 2.0

    class State(enum.Enum):
        our_goal = 0
        their_goal = 1
        pause = 2

    def __init__(self):
        super().__init__(continuous=True)

        self.side_start = time.time()

        for state in RepeatedTurningLineUpLengthwise.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(
            behavior.Behavior.State.start,
            RepeatedTurningLineUpLengthwise.State.their_goal, lambda: True,
            'immediately')

        self.add_transition(
            RepeatedTurningLineUpLengthwise.State.our_goal,
            RepeatedTurningLineUpLengthwise.State.pause, lambda: self.
            subbehavior_with_name('LineUp').state == behavior.Behavior.State.
            completed and time.time() - self.side_start > 1,
            'made it to our_goal')

        self.add_transition(
            RepeatedTurningLineUpLengthwise.State.their_goal,
            RepeatedTurningLineUpLengthwise.State.pause, lambda: self.
            subbehavior_with_name('LineUp').state == behavior.Behavior.State.
            completed and time.time() - self.side_start > 1,
            'made it to their_goal')

        self.add_transition(
            RepeatedTurningLineUpLengthwise.State.pause,
            RepeatedTurningLineUpLengthwise.State.their_goal, lambda: (
                time.time() - self.pause_start_time
            ) > RepeatedTurningLineUpLengthwise.Pause and self.prev_side ==
            RepeatedTurningLineUpLengthwise.State.our_goal, 'pause over')
        self.add_transition(
            RepeatedTurningLineUpLengthwise.State.pause,
            RepeatedTurningLineUpLengthwise.State.our_goal, lambda: (
                time.time() - self.pause_start_time
            ) > RepeatedTurningLineUpLengthwise.Pause and self.prev_side ==
            RepeatedTurningLineUpLengthwise.State.their_goal, 'pause over')

    def on_enter_our_goal(self):
        self.side_start = time.time()
        self.add_subbehavior(
            tactics.line_up.LineUp(self.generate_line(-1)), 'LineUp')

    def on_exit_our_goal(self):
        self.remove_subbehavior('LineUp')
        self.prev_side = RepeatedTurningLineUpLengthwise.State.our_goal

    def on_enter_their_goal(self):
        self.side_start = time.time()
        self.add_subbehavior(
            tactics.line_up.LineUp(self.generate_line(1)), 'LineUp')

    def on_exit_their_goal(self):
        self.remove_subbehavior('LineUp')
        self.prev_side = RepeatedTurningLineUpLengthwise.State.their_goal

    def on_enter_pause(self):
        self.pause_start_time = time.time()

    def execute_running(self):
        for lineup_bhvr in self.all_subbehaviors():
            for bhvr in lineup_bhvr.all_subbehaviors():
                if (bhvr.robot != None):
                    bhvr.robot.face(robocup.Point(0, 0))

    # y_multiplier is a 1 or -1 to indicate which side of the field to be on
    # 1 is their goal, -1 is our goal
    def generate_line(self, y_multiplier):
        y = ((constants.Field.Length / 2 - constants.Field.GoalWidth -
              constants.Robot.Radius * 2) * y_multiplier) + (
                  constants.Field.Length / 2)
        x_start = -0.8
        line = robocup.Segment(
            robocup.Point(constants.Robot.Radius + x_start, y),
            robocup.Point((constants.Robot.Radius * 2 + 0.1) * 6 + x_start, y))
        return line
