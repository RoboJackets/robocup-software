import stp.situation as situation
import stp
import stp.rc as rc
import rj_gameplay.situation.decision_tree.analyzer as analyzer
import rj_gameplay.situation.decision_tree.plays as situations
import rj_gameplay.play as plays
from rj_gameplay.play import basic122, basic_defense, basic_scramble, defensive_clear, defend_restart, restart
from typing import Tuple, Dict

#TODO: Put new plays into the dict properly
#TODO: Create different dictionaries for different playbooks
PLAY_DICT = {}
PLAY_DICT[situations.Kickoff] = [basic_scramble.Scramble]
PLAY_DICT[situations.DefendRestartOffensive] = [defend_restart.DefendRestart]
PLAY_DICT[situations.DefendRestartMidfield] = [defend_restart.DefendRestart]
PLAY_DICT[situations.DefendRestartDefensive] = [defend_restart.DefendRestart]
PLAY_DICT[situations.Clear] = [defensive_clear.DefensiveClear]
PLAY_DICT[situations.DefendClear] = [defensive_clear.DefensiveClear]
PLAY_DICT[situations.DefendGoal] = [basic_defense.BasicDefense]
PLAY_DICT[situations.MidfieldClear] = [defensive_clear.DefensiveClear]
PLAY_DICT[situations.AttackGoal] = [basic122.Basic122]
PLAY_DICT[situations.OffensiveScramble] = [basic_scramble.Scramble]
PLAY_DICT[situations.MidfieldScramble] = [basic_scramble.Scramble]
PLAY_DICT[situations.DefensiveScramble] = [basic_scramble.Scramble]
PLAY_DICT[situations.SaveBall] = [basic_defense.BasicDefense]
PLAY_DICT[situations.SaveShot] = [basic_defense.BasicDefense]
PLAY_DICT[situations.OffensivePileup] = [basic_scramble.Scramble]
PLAY_DICT[situations.MidfieldPileup] = [basic_scramble.Scramble]
PLAY_DICT[situations.DefensivePileup] = [basic_scramble.Scramble]
PLAY_DICT[situations.MidfieldDefendClear] = [defensive_clear.DefensiveClear]
PLAY_DICT[situations.Shootout] = [basic_defense.BasicDefense]
PLAY_DICT[situations.DefendShootout] = [basic_defense.BasicDefense]
PLAY_DICT[situations.Penalty] = [basic_defense.BasicDefense]
PLAY_DICT[situations.DefendPenalty] = [basic_defense.BasicDefense]
PLAY_DICT[situations.OffensiveKick] = [basic_defense.BasicDefense]
PLAY_DICT[situations.DefensiveKick] = [basic_defense.BasicDefense]
PLAY_DICT[situations.MidfieldKick] = [basic_defense.BasicDefense]
PLAY_DICT[situations.GoalieClear] = [defensive_clear.DefensiveClear]

class BasicPlaySelector(situation.IPlaySelector):

    def __init__(self):
        self.analyzer = analyzer.Analyzer()
        self.curr_situation = None
        self.curr_play = None

    def select(self, world_state: rc.WorldState) -> Tuple[situation.ISituation, stp.play.IPlay]:
        if world_state.game_info is None and self.curr_play is None:
            return (self.curr_situation, basic_defense.BasicDefense())
        if world_state.game_info is not None:
            plays_selection = []
            self.curr_situation = self.analyzer.analyze_situation(world_state)
            for sit, possible_plays in PLAY_DICT.items():
                if isinstance(self.curr_situation, sit):
                    plays_selection = possible_plays
            if plays_selection:
                self.curr_play = plays_selection[0]()
                return (self.curr_situation, self.curr_play)
            else:
                return (self.curr_situation, self.curr_play)
        else:
            return (self.curr_situation, self.curr_play)
