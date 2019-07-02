import play
import behavior
import constants
import robocup
import tactics.adaptive_defense as ad
import main
import skills

class TestAdaptiveDefense(play.Play):
    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running, lambda: True,
            'now')

        defense = ad.AdaptiveDefense()
        self.add_subbehavior(defense, "adaptive_defense")