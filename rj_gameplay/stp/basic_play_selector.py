import stp.situation as situation
import stp
import stp.rc as rc
import rj_gameplay.situation.decision_tree.analyzer as analyzer
from typing import Tuple

from rj_gameplay.play import line_up, passing_tactic_play

class BasicPlaySelector(situation.IPlaySelector):

	def __init__(self):
		self.analyzer = analyzer.Analyzer()
		self.curr_situation = None
		self.curr_play = None

	def select(self, world_state: rc.WorldState) -> Tuple[situation.ISituation, stp.play.IPlay]:
		if world_state.game_info is not None:
			self.curr_situation = self.analyzer.analyze_situation(world_state)
		print(self.curr_situation)
		return (self.curr_situation, passing_tactic_play.PassPlay())



