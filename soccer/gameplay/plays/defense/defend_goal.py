import main
import robocup
import behavior
import constants
import enum

import standard_play
import tactics.positions.submissive_defender as submissive_defender

## Play that uses submissive defenders and wingers to defend
#  an attack close to our goal
class DefendGoal(standard_play.StandardPlay):

    def __init__(self, num_defenders=3, num_wingers=3):
        super().__init__(continuous=True)

        self.num_defenders = num_defenders
        self.num_wingers = num_wingers

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running, lambda: True,
            'Immediately')

        for i in range(num_defenders):
            defender = submissive_defender.SubmissiveDefender()
            self.add_subbehavior(defender, 'defender' + str(i), required=True)

        # for i in range(num_wingers):
        #     winger = winger.WingerDefense()
        #     self.add_subbehavior('winger' + str(i), winger)