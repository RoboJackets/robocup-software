import standard_play
import behavior
import skills
import main
import situational_play_selection

## Simplistic play to handle defend clear situation
#  Mimicks basic122 offense. One pivot kick and two marking robots
#
class BaiscDefendClear(standard_play.StandardPlay):

    _situationList = [
        situational_play_selection.SituationalPlaySelector.Situation.DEFEND_CLEAR,
    ] # yapf: disable

    def __init__(self, num_supports=2):
        super().__init__(continuous=False)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'Immediately transition to running')

        best_opps_to_mark = self.closest_opps_to_our_goal(num_supports)
        self.add_subbehavior(skills.pivot_kick.PivotKick(), 'main')
        for i in range(num_supports):
            mark_skill = skills.mark.Mark()
            mark_skill.mark_robot = best_opps_to_mark[i]
            self.add_subbehavior(mark_skill, 'mark' + str(i))

    ## Returns the robots that are closest to our goal
    #
    # @param num_bots number of robots to return
    # @returns bots closest num_bots opponent robots to our goal
    def closest_opps_to_our_goal(self, num_bots):
        opp_bots = [bot for bot in main.their_robots()]
        return sorted(opp_bots, key=lambda bot: bot.pos.y)[:num_bots]
