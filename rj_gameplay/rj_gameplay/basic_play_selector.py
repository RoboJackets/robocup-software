import stp.situation as situation
import stp
import stp.rc as rc
import rj_gameplay.situation.decision_tree.analyzer as analyzer
import rj_gameplay.situation.decision_tree.plays as situations
import rj_gameplay.play as plays
from rj_gameplay.play import line_up, basic_defense, keepaway, basic122
from typing import Tuple, Optional

# TODO: Create different dictionaries for different playbooks
"""
# TODO: write these plays (based on the old play list)
penalty_defense,
penalty_offense,
prep_penalty_offense,
defend_restart,
restart,
kickoff_play,
"""
# for now, all situations that don't have a play use line_up as placeholder
# Dict: {situation.type : play.type}
PLAY_DICT = {
    situations.PrepareKickoff: line_up.LineUp,
    situations.Kickoff: basic122.Basic122,
    situations.DefendKickoff: line_up.LineUp,
    # defend_restart related
    situations.DefendRestartOffensive: line_up.LineUp,
    situations.DefendRestartMidfield: line_up.LineUp,
    situations.DefendRestartDefensive: line_up.LineUp,
    situations.DefendRestartOffensiveDirect: line_up.LineUp,
    situations.DefendRestartMidfieldDirect: line_up.LineUp,
    situations.DefendRestartDefensiveDirect: line_up.LineUp,
    # clear related
    situations.Clear: line_up.LineUp,
    situations.DefendClear: line_up.LineUp,
    situations.MidfieldClear: line_up.LineUp,
    situations.MidfieldDefendClear: line_up.LineUp,
    # defense related
    situations.DefendGoal: basic_defense.BasicDefense,
    # offense related
    situations.AttackGoal: basic122.Basic122,
    # scramble
    situations.OffensiveScramble: basic122.Basic122,
    situations.MidfieldScramble: basic122.Basic122,
    situations.DefensiveScramble: basic_defense.BasicDefense,
    # ???
    situations.SaveBall: basic_defense.BasicDefense,
    situations.SaveShot: basic_defense.BasicDefense,
    # pileup
    situations.OffensivePileup: basic122.Basic122,
    situations.MidfieldPileup: basic122.Basic122,
    situations.DefensivePileup: basic122.Basic122,
    # shootout = penalty?
    situations.Shootout: line_up.LineUp,
    situations.PrepareShootout: line_up.LineUp,
    situations.DefendShootout: line_up.LineUp,
    situations.PrepareDefendShootout: line_up.LineUp,
    # penalty = shootout?
    situations.Penalty: line_up.LineUp,
    situations.PreparePenalty: line_up.LineUp,
    situations.DefendPenalty: line_up.LineUp,
    situations.PrepareDefendPenalty: line_up.LineUp,
    # ???
    situations.OffensiveKick: line_up.LineUp,
    situations.DefensiveKick: line_up.LineUp,
    situations.MidfieldKick: line_up.LineUp,
    # ???
    situations.OffensiveKickDirect: line_up.LineUp,
    situations.DefensiveKickDirect: line_up.LineUp,
    situations.MidfieldKickDirect: line_up.LineUp,
    # ???
    situations.GoalieClear: line_up.LineUp,
    situations.Stop: line_up.LineUp,
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
