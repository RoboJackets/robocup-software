import main
import robocup
import behavior
import constants
import enum

import standard_play
import evaluation
import situational_play_selection
import skills.pivot_kick as pivot_kick

## Makes a single robot chip up the field
# 
class BasicDefendRestartOffensive(standard_play.StandardPlay):
        
    _situationList = [
        situational_play_selection.SituationalPlaySelector.Situation.DEFEND_RESTART_DEFENSIVE
    ] # yapf: disable

    def __init__(self):
        super().__init__(continuous = False)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'Immediately transition to running')

        kick = pivot_kick.PivotKick()
        kick.aim_target_point = robocup.Point(main.ball().pos.x, constants.Field.Length / 2)
        kick.use_chippter = True

        # consider adding more complicated functionality
        self.add_subbehavior(kick, 'chip_to_center')

        self.add_transition(behavior.Behavior.State.running, behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name('chip_to_center').is_done_running(),
            'kick completed')

    def on_exit_running(self):
        self.remove_all_subbehaviors()