import stp.situation as situation
import stp
import stp.rc as rc
import rj_gameplay.situation.decision_tree.analyzer as analyzer
import rj_gameplay.situation.decision_tree.plays as situations
import rj_gameplay.play as plays
from rj_gameplay.play import defensive_clear, wall_ball
from typing import Tuple, Dict

#TODO: Put new plays into the dict properly
#TODO: Create different dictionaries for different playbooks
PLAY_DICT = {}
PLAY_DICT[situations.Kickoff] = [plays.line_up.LineUp]
PLAY_DICT[situations.DefendRestartOffensive] = []
PLAY_DICT[situations.DefendRestartMidfield] = []
PLAY_DICT[situations.DefendRestartDefensive] = []
PLAY_DICT[situations.Clear] = [plays.passing_tactic_play.PassPlay]
PLAY_DICT[situations.DefendClear] = [defensive_clear.DefensiveClear]
PLAY_DICT[situations.DefendGoal] = [plays.wall_ball.WallBall]
PLAY_DICT[situations.MidfieldClear] = []
PLAY_DICT[situations.AttackGoal] = [plays.passing_tactic_play.PassPlay]
PLAY_DICT[situations.OffensiveScramble] = [plays.passing_tactic_play.PassPlay]
PLAY_DICT[situations.MidfieldScramble] = [plays.passing_tactic_play.PassPlay]
PLAY_DICT[situations.DefensiveScramble] = [plays.passing_tactic_play.PassPlay]
PLAY_DICT[situations.SaveBall] = []
PLAY_DICT[situations.SaveShot] = []
PLAY_DICT[situations.OffensivePileup] = [plays.passing_tactic_play.PassPlay]
PLAY_DICT[situations.MidfieldPileup] = [plays.passing_tactic_play.PassPlay]
PLAY_DICT[situations.DefensivePileup] = [plays.passing_tactic_play.PassPlay]
PLAY_DICT[situations.MidfieldDefendClear] = [plays.defensive_clear.DefensiveClear]
PLAY_DICT[situations.Shootout] = []
PLAY_DICT[situations.DefendShootout] = []
PLAY_DICT[situations.Penalty] = []
PLAY_DICT[situations.DefendPenalty] = []
PLAY_DICT[situations.OffensiveKick] = []
PLAY_DICT[situations.DefensiveKick] = []
PLAY_DICT[situations.MidfieldKick] = []
PLAY_DICT[situations.GoalieClear] = []

class BasicPlaySelector(situation.IPlaySelector):

    def __init__(self):
        self.analyzer = analyzer.Analyzer()
        self.curr_situation = None
        self.curr_play = None

    def select(self, world_state: rc.WorldState) -> Tuple[situation.ISituation, stp.play.IPlay]:
        if world_state.game_info is None and self.curr_play is None:
            return (self.curr_situation, plays.passing_tactic_play.PassPlay())
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
