import main
import robocup
import behavior
import constants
import enum

import standard_play
import tactics.positions.submissive_goalie as submissive_goalie
import tactics.positions.submissive_defender as submissive_defender
import evaluation.opponent as eval_opp
import tactics.positions.wing_defender as wing_defender
import skills.mark as mark
import tactics.wall as wall
import situational_play_selection


## Restart that uses standard defense and uses the remaining
#  robots to form a wall
#
class BasicDefendRestartDefensive(standard_play.StandardPlay):

    _situationList = [
        situational_play_selection.SituationalPlaySelector.Situation.DEFEND_RESTART_DEFENSIVE
    ] # yapf: disable


    def __init__(self, num_defenders=2):
        super().__init__(continuous=True)

        self.num_defenders = num_defenders

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'Immediately')

        self.add_subbehavior(wall.Wall(), 'wall', required=False)


