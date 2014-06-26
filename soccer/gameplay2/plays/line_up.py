import play
import behavior
import tactics.line_up
import robocup


class LineUp(play.Play):

    def __init__(self):
        super().__init__(continuous=False)

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')
        self.add_transition(behavior.Behavior.State.running,
            behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name("LineUp").State is behavior.Behavior.State.completed,
            'all robots reach target positions')
        self.add_transition(behavior.Behavior.State.completed,
            behavior.Behavior.State.running,
            lambda: self.subbehavior_with_name("LineUp").State is behavior.Behavior.State.running,
            'robots arent lined up')

        self.add_subbehavior(tactics.line_up.LineUp(), name="LineUp", required=True)
