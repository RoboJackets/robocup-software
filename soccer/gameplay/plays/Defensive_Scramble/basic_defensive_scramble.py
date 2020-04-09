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
import skills.capture as capture
import situational_play_selection


## Play that uses submissive defenders and wingers to defend
#  an attack close to our goal and cause one robot to capture.
#
#  By default, we will use standard defense (two submissive
#  defenders, one goalie) and two wing defense robots. The
#  remaining robot will mark the highest threat robot.
class BasicDefensiveScramble(standard_play.StandardPlay):

    _situationList = [
        situational_play_selection.SituationalPlaySelector.Situation.DEFENSIVE_SCRAMBLE
    ] # yapf: disable

    def __init__(self, num_defenders=2, num_wingers=2):
        super().__init__(continuous=True)

        self.num_defenders = num_defenders
        self.num_wingers = num_wingers

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'Immediately')

        self.add_subbehavior(capture.Capture(), 'ball capturer')

        # Keep track of which robots are currently being defended
        self.defended = {}
        for i in range(len(main.their_robots())):
            self.defended[i] = False

        for i in range(self.num_wingers):
            self.add_subbehavior(wing_defender.WingDefender(),
                                 'winger' + str(i))

    def execute_running(self):

        for i in range(self.num_wingers):
            bhvr = self.subbehavior_with_name('winger' + str(i))

            # Control which robots are being defended by wing defender
            for i in range(len(main.their_robots())):
                bot = main.their_robots()[i]
                if not eval_opp.is_marked(bot.pos) and not self.defended[i]:
                    bhvr.mark_robot = bot
                    self.defended[i] = True

        for bot in main.their_robots():
            if not eval_opp.is_marked(bot.pos):
                self.defended[i] = False
