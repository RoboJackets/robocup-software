import play
import behavior
import robocup
import main
import constants
import tactics.line_up
import tactics.our_placement


# one robot places the ball, the others just line up and wait
class Placement(play.Play):
    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        self.add_transition(
            behavior.Behavior.State.running, behavior.Behavior.State.completed,
            lambda: self.placer.is_done_running(), 'when placer finishes.')
        
        print("SELF.PLACER")
        self.placer = tactics.our_placement.OurPlacement()
        self.add_subbehavior(self.placer, 'placer', required=True, priority=90)
        print("EXTERNAL ADD SUBBEHAVIOR COMPLETE")
        line = robocup.Segment(robocup.Point(1.5, 1), robocup.Point(1.5, 2.5))
        line_up = tactics.line_up.LineUp(line)

        

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if gs.is_our_placement() else float("inf")

    @classmethod
    def is_restart(cls):
        return True
