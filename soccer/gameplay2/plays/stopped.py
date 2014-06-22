import play
import behavior

# when we get the Stopped command from the referee,
# we run this play.  See the rules to see what we're allowed to do while the game is stopped
class Stopped(play.Play):
    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')
