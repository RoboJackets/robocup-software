import stp.situation as situation
import stp
import stp.rc as rc
import rj_gameplay.situation.decision_tree.analyzer as analyzer
import rj_gameplay.situation.decision_tree.plays as situations
import rj_gameplay.play as plays
from rj_gameplay.play import (
    basic122,
    basic_defense,
    defensive_clear,
    defend_restart,
    restart,
    kickoff_play,
    penalty_defense,
    penalty_offense,
    prep_penalty_offense,
)
from typing import Tuple, Optional

# TODO: Put new plays into the dict properly
# TODO: Create different dictionaries for different playbooks
PLAY_DICT = {
    situations.PrepareKickoff: kickoff_play.PrepareKickoffPlay,
    situations.Kickoff: basic122.Basic122,
    situations.DefendKickoff: kickoff_play.DefendKickoffPlay,
    situations.DefendRestartOffensive: defend_restart.DefendRestart,
    situations.DefendRestartMidfield: defend_restart.DefendRestart,
    situations.DefendRestartDefensive: defend_restart.DefendRestart,
    situations.DefendRestartOffensiveDirect: defend_restart.DefendRestart,
    situations.DefendRestartMidfieldDirect: defend_restart.DefendRestart,
    situations.DefendRestartDefensiveDirect: defend_restart.DefendRestart,
    situations.Clear: defensive_clear.DefensiveClear,
    situations.DefendClear: defensive_clear.DefensiveClear,
    situations.DefendGoal: basic_defense.BasicDefense,
    situations.MidfieldClear: defensive_clear.DefensiveClear,
    situations.AttackGoal: basic122.Basic122,
    situations.OffensiveScramble: basic122.Basic122,
    situations.MidfieldScramble: basic122.Basic122,
    situations.DefensiveScramble: defensive_clear.DefensiveClear,
    situations.SaveBall: basic_defense.BasicDefense,
    situations.SaveShot: basic_defense.BasicDefense,
    situations.OffensivePileup: basic122.Basic122,
    situations.MidfieldPileup: basic122.Basic122,
    situations.DefensivePileup: basic122.Basic122,
    situations.MidfieldDefendClear: defensive_clear.DefensiveClear,
    situations.Shootout: penalty_offense.PenaltyOffense,
    situations.PrepareShootout: prep_penalty_offense.PreparePenaltyOffense,
    situations.DefendShootout: penalty_defense.PenaltyDefense,
    situations.PrepareDefendShootout: penalty_defense.PreparePenaltyDefense,
    situations.Penalty: penalty_offense.PenaltyOffense,
    situations.PreparePenalty: prep_penalty_offense.PreparePenaltyOffense,
    situations.DefendPenalty: penalty_defense.PenaltyDefense,
    situations.PrepareDefendPenalty: penalty_defense.PreparePenaltyDefense,
    situations.OffensiveKick: restart.RestartPlay,
    situations.DefensiveKick: restart.RestartPlay,
    situations.MidfieldKick: restart.RestartPlay,
    situations.OffensiveKickDirect: restart.DirectRestartPlay,
    situations.DefensiveKickDirect: restart.DirectRestartPlay,
    situations.MidfieldKickDirect: restart.DirectRestartPlay,
    situations.GoalieClear: defensive_clear.DefensiveClear,
    situations.Stop: defend_restart.DefendRestart,
}


class BasicPlaySelector(situation.IPlaySelector):
    """Play selector that returns a play based on the situation from analyzer.

    Currently configured to take in only one play per situation. Situation to play mapping is determined in PLAY_DICT constant above. (This means using this class requires importing the whole file, rather than just the class.)
    """

    def __init__(self):
        self.analyzer = analyzer.Analyzer()
        self.curr_situation = None
        self.curr_play = None

    def select(
        self, world_state: rc.WorldState
    ) -> Tuple[Optional[situation.ISituation], stp.play.IPlay]:

        if world_state.game_info is None and self.curr_play is None:
            return (self.curr_situation, basic_defense.BasicDefense())

        if world_state.game_info is not None:
            self.curr_situation = self.analyzer.analyze_situation(world_state)
            for sit, play in PLAY_DICT.items():
                if isinstance(self.curr_situation, sit):
                    self.curr_play = play()

        return (self.curr_situation, self.curr_play)
