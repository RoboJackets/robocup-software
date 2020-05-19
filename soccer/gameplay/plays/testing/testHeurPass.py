import main
import robocup
import behavior
import constants
import enum

import standard_play
import evaluation
import situational_play_selection
import tactics.coordinated_pass
import skills.move
import skills.capture
import random
import tactics.pass_to_point_heuristic

#Based on clear sitation
#    Have ball in defense
#   Want to Kick ball up field and get other robots in position
#


##
# A basic clearing play written in responce to the fact that we didn't have a clearing play
#
# It needs to be renamed as "clear" is the name of the situation, and each play that fufills it
# needs to have a unique name beyond that
#
class TestHeuristic(standard_play.StandardPlay):

    def __init__(self):
        super().__init__(continuous=False)

    def on_enter_start(self):
        self.heuristic = tactics.pass_to_point_heuristic.PassToPointHeuristic(1,2)
        self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(self.heuristic), 'pass')