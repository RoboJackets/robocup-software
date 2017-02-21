import behavior
import robocup
import main
import enum
import constants
import tactics.line_up
import tactics.tune_pid
import skills.move
import play


# plan is for this to eventually loop through all robots and tune one at a time
class pid(play.Play):
    class State(enum.Enum):
        prep = 1 #go to the start point, line up other bots
        testing = 2  # Drive a test line

    def __init__(self):
        super().__init__(continuous=True)

        for state in pid.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            pid.State.prep, lambda: True,
                            'immediately')

        self.add_transition(pid.State.prep, pid.State.testing, lambda: self.has_subbehavior_with_name('move2') and self.subbehavior_with_name('move2').state == behavior.Behavior.State.completed and self.subbehavior_with_name('move2').robot.vel.mag() < .05 ,'finished moving')


        self.add_transition(pid.State.testing, behavior.Behavior.State.completed,
                            lambda: self.check_continue(), 'command changed')

        self.positions=[]

    def create_lineup(self):
        xsize = constants.Field.Width / 2 - .5

        return robocup.Segment(
            robocup.Point(xsize, .25), robocup.Point(xsize, 1.5))


    def check_continue(self):
        return False

    def on_enter_prep(self):
        xsize = constants.Field.Width/2
        move = skills.move.Move(robocup.Point((-xsize/2)+.1,2))
        self.add_subbehavior(move, 'move1', required=True, priority=100)

        line_up = tactics.line_up.LineUp(self.create_lineup())
        self.add_subbehavior(line_up, 'line_up', required=True, priority=80)

    def execute_prep(self):
        if self.has_subbehavior_with_name('move1') and self.subbehavior_with_name('move1').state == behavior.Behavior.State.completed:
            self.remove_subbehavior('move1')
            xsize = constants.Field.Width/2
            move2 = skills.move.Move(robocup.Point(-xsize+.1,2))
            self.add_subbehavior(move2, 'move2', required=True, priority=100)

    def on_exit_prep(self):
        self.remove_subbehavior('move2')

    def on_enter_testing(self):
        tune = tactics.tune_pid.Tune_pid()
        self.add_subbehavior(tune, 'tune', required=True, priority=100)

    def on_exit_testing(self):
        self.remove_subbehavior('tune')

    @classmethod
    def score(cls):
        return 10;

    @classmethod
    def handles_goalie(cls):
        return True
