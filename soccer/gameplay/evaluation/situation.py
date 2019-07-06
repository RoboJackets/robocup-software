import standard_play
import behavior
import skills
import tactics
import robocup
import constants
import main
import evaluation.passing
import evaluation.shooting
import math
import numpy as np

HALF_LENGTH = 5

def goal_per_minute_required():
	gs = main.system_state().game_state
	time_left = gs.seconds_remaining / 60
	if gs.is_first_half() or gs.is_overtime1():
		time_left += HALF_LENGTH
	return (gs.their_score - gs.our_score) / time_left

def we_are_winning():
	gs = main.system_state().game_state
	return gs.our_score > gs.their_score 

def we_are_losing():
	gs = main.system_state().game_state
	return gs.our_score < gs.their_score 