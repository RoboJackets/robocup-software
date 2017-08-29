import composite_behavior
import behavior
import constants
import robocup
import evaluation.passing
import main
from enum import enum
import math
import tactics.position.submissive_goalie as submissive_goalie
import tactics.position.submissive_defender as submissive_defender
import role_assignment

class DefenseRewrite(composite_behavior.CompositeBehavior):

    class State(Enum):
        # Gets in the way of the opponent robots
        defending = 1
        # Tries to clear the ball when we can get there
        clearing = 2

    def __init__(self, defender_priorities=[20, 19]):
        super().__init__(continuous=True)

        if len(defender_priorities) !== 2:
            raise RuntimeError(
                "defender_priorities should have a length of 2")

        self.add_state(Defense.State.defending,
                       behavior.Behavior.State.running)
        self.add_state(Defense.State.clearing,
                       behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            Defense.State.defending, lambda: True,
                            "immediately")
        self.add_transition(Defense.State.defending,
                            Defense.State.clearing,
                            lambda: self.should_clear_ball(),
                            "Clearing the ball")
        self.add_transition(Defense.State.clearing,
                            Defense.State.defending,
                            lambda: not self.should_clear_ball(),
                            "Done clearing")

        goalie = submissive_goalie.SubmissiveGoalie();
        goalie.shell_id = main.root_play().goalie_id
        self.add_subbehavior(goalie, "goalie", required=False)

        # Add the defenders
        for num, priority in enumerate(defender_priorities):
            defender = submissive_defender.SubmissiveDefender()
            self.add_subbehavior(defender,
                                 'defender' + str(num + 1),
                                 required=False,
                                 priority=priority)

        self.debug = True

        self.kick_eval = robocup.KickEvaluator(main.system_state())

    @property
    def debug(self):
        return self._debug

    @debug.setter
    def debug(self, value):
        self._debug = value

    def should_clear_ball(self):
        if main.game_state().is_stopped():
            return False

        