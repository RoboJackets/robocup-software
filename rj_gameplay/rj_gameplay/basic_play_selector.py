import stp.situation as situation
import stp
import stp.rc as rc
import rj_gameplay.situation.decision_tree.analyzer as analyzer
import rj_gameplay.situation.decision_tree.plays as situations
import rj_gameplay.play as plays
from rj_gameplay.play import basic122, basic_defense, defensive_clear, defend_restart, restart, kickoff_play, penalty_defense, penalty_offense, prep_penalty_offense
from typing import Tuple, Dict

#TODO: Put new plays into the dict properly
#TODO: Create different dictionaries for different playbooks
PLAY_DICT = {}
PLAY_DICT[situations.PrepareKickoff] = [kickoff_play.PrepareKickoffPlay]
PLAY_DICT[situations.Kickoff] = [basic122.Basic122]
PLAY_DICT[situations.DefendKickoff] = [kickoff_play.DefendKickoffPlay]
PLAY_DICT[situations.DefendRestartOffensive] = [defend_restart.DefendRestart]
PLAY_DICT[situations.DefendRestartMidfield] = [defend_restart.DefendRestart]
PLAY_DICT[situations.DefendRestartDefensive] = [defend_restart.DefendRestart]
PLAY_DICT[situations.DefendRestartOffensiveDirect] = [defend_restart.DefendRestart]
PLAY_DICT[situations.DefendRestartMidfieldDirect] = [defend_restart.DefendRestart]
PLAY_DICT[situations.DefendRestartDefensiveDirect] = [defend_restart.DefendRestart]
PLAY_DICT[situations.Clear] = [defensive_clear.DefensiveClear]
PLAY_DICT[situations.DefendClear] = [defensive_clear.DefensiveClear]
PLAY_DICT[situations.DefendGoal] = [basic_defense.BasicDefense]
PLAY_DICT[situations.MidfieldClear] = [defensive_clear.DefensiveClear]
PLAY_DICT[situations.AttackGoal] = [basic122.Basic122]
PLAY_DICT[situations.OffensiveScramble] = [basic122.Basic122]
PLAY_DICT[situations.MidfieldScramble] = [basic122.Basic122]
PLAY_DICT[situations.DefensiveScramble] = [defensive_clear.DefensiveClear]
PLAY_DICT[situations.SaveBall] = [basic_defense.BasicDefense]
PLAY_DICT[situations.SaveShot] = [basic_defense.BasicDefense]
PLAY_DICT[situations.OffensivePileup] = [basic122.Basic122]
PLAY_DICT[situations.MidfieldPileup] = [basic122.Basic122]
PLAY_DICT[situations.DefensivePileup] = [basic122.Basic122]
PLAY_DICT[situations.MidfieldDefendClear] = [defensive_clear.DefensiveClear]
PLAY_DICT[situations.Shootout] = [penalty_offense.PenaltyOffense]
PLAY_DICT[situations.PrepareShootout] = [prep_penalty_offense.PreparePenaltyOffense]
PLAY_DICT[situations.DefendShootout] = [penalty_defense.PenaltyDefense]
PLAY_DICT[situations.PrepareDefendShootout] = [penalty_defense.PreparePenaltyDefense]
PLAY_DICT[situations.Penalty] = [penalty_offense.PenaltyOffense]
PLAY_DICT[situations.PreparePenalty] = [prep_penalty_offense.PreparePenaltyOffense]
PLAY_DICT[situations.DefendPenalty] = [penalty_defense.PenaltyDefense]
PLAY_DICT[situations.PrepareDefendPenalty] = [penalty_defense.PreparePenaltyDefense]
PLAY_DICT[situations.OffensiveKick] = [restart.RestartPlay]
PLAY_DICT[situations.DefensiveKick] = [restart.RestartPlay]
PLAY_DICT[situations.MidfieldKick] = [restart.RestartPlay]
PLAY_DICT[situations.OffensiveKickDirect] = [restart.DirectRestartPlay]
PLAY_DICT[situations.DefensiveKickDirect] = [restart.DirectRestartPlay]
PLAY_DICT[situations.MidfieldKickDirect] = [restart.DirectRestartPlay]
PLAY_DICT[situations.GoalieClear] = [defensive_clear.DefensiveClear]
PLAY_DICT[situations.Stop] = [defend_restart.DefendRestart]

class BasicPlaySelector(situation.IPlaySelector):
    """Play selector that returns a play based on the situation from analyzer.

    Currently configured to take in only one play per situation. Situation to play mapping is determined in PLAY_DICT constant above. (This means using this class requires importing the whole file, rather than just the class.)
    """

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
