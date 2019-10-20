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
import tactics.defense

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

        goalie = submissive_goalie.SubmissiveGoalie()
        goalie.shell_id = main.root_play().goalie_id
        # self.add_subbehavior(goalie, "goalie", required=True)

        # for i in range(num_defenders):
        #     defender = submissive_defender.SubmissiveDefender()
        #     self.add_subbehavior(defender, 'defender' + str(i))

        # self.threat_list = list(map(lambda threat: threat[0], eval_opp.get_threat_list(self.all_subbehaviors())))
        # self.threat_list = eval_opp.get_threat_list(self.all_subbehaviors())

        self.add_subbehavior(tactics.defense.Defense(), 'defense', required=False)

        self.wingers = {}
        for i in range(self.num_wingers):
            self.wingers['winger' + str(i)] = wing_defender.WingDefender()
        for i in range(self.num_wingers):
            self.add_subbehavior(wing_defender.WingDefender(), 'winger' + str(i))


    def execute_running(self):
        # print(self.threat_list[0])

        # for point, threat, roboot in self.threat_list:
            # print(point)
        # for bhvr in self.all_subbehaviors():
        #     print(bhvr)

        for i in range(self.num_wingers):
            bhvr = self.subbehavior_with_name('winger' + str(i))

            for bot in main.their_robots():
                if not eval_opp.is_marked(bot.pos):
                    bhvr.mark_robot = bot
        
        # for name, bhvr in self.wingers:
        #     for bot in main.their_robots():
        #         if not eval_opp.is_marked(bot.pos):
        #             bhvr.mark_robot(bot)
        #             self.add_subbehavior(bhvr, name)