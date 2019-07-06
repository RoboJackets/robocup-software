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

# Returns a list of points a specified radius from the origin point
# 0 is going striaght to the right and angles go up (towards opponent goal) from there.
def get_radius_points(start_point, r=1, n=50, from_rad=0, to_rad=180):
	return [robocup.Point(math.cos(angle), math.sin(angle))*r + start_point for angle in math.pi/180*np.linspace(from_rad, to_rad, n)]

def get_opponent_distances_to_point(point):
	return [(rob.pos - point).mag() for rob in main.system_state().their_robots if rob.visible]

def get_our_distances_to_point(point):
	return [(rob.pos - point).mag() for rob in main.system_state().our_robots if rob.visible]

def get_closest_opponent_distance_to_point(point):
	tmp = [((rob.pos - point).mag() , rob) for rob in main.system_state().their_robots if rob.visible]
	if len(tmp)==0:
		return float('inf'), None
	else:
		return sorted(tmp, key=lambda x: x[0])[0]

def get_closest_teammate_distance_to_point(point):
	tmp = [((rob.pos - point).mag() , rob) for rob in main.system_state().our_robots if rob.visible]
	if len(tmp)==0:
		return float('inf'), None
	else:
		return sorted(tmp, key=lambda x: x[0])[0]

# 
# downfield robots are ones closer to the opponent goal the parameter point
# @param point: robocup.Point
def get_closest_downfield_opponent_distance_to_point(point):
	tmp = [((rob.pos - point).mag() , rob) for rob in main.system_state().their_robots if rob.pos.y >= point.y and rob.visible]
	if len(tmp)==0:
		return float('inf'), None
	else:
		return sorted(tmp, key=lambda x: x[0])[0]

# 
# Upfield robots are ones closer to our goal the parameter point
# @param point: robocup.Point
def get_closest_upfield_opponent_distance_to_point(point):
	tmp = [((rob.pos - point).mag() , rob) for rob in main.system_state().their_robots if rob.pos.y < point.y and rob.visible]
	if len(tmp)==0:
		return float('inf'), None
	else:
		return sorted(tmp, key=lambda x: x[0])[0]

# 
# Upfield robots are ones closer to our goal the parameter point
# @param point: robocup.Point
def get_closest_upfield_teammate_distance_to_point(point):
	tmp = [((rob.pos - point).mag() , rob) for rob in main.system_state().our_robots if rob.pos.y < point.y and rob.visible]
	if len(tmp)==0:
		return float('inf'), None
	else:
		return sorted(tmp, key=lambda x: x[0])[0]

def outer_third_linear_penalty_func(point):
	"""
	Returns a multiplicative penalty (assuming higher is good, outputs 0->1) 
	based on where the point is width wise on the field 
	"""
	width_sixth = constants.Field.Width/6
	if abs(point.x) <= width_sixth:
		return 1
	else:
		return 1 - ((abs(point.x) - width_sixth)/(2*width_sixth))

def outer_third_halflinear_penalty_func(point):
	"""
	Returns a multiplicative penalty (assuming higher is good, outputs 0->1) 
	based on where the point is width wise on the field 
	"""
	width_sixth = constants.Field.Width/6
	if abs(point.x) <= width_sixth:
		return 1
	else:
		return 1 - ((abs(point.x) - width_sixth)/(4*width_sixth))

def dot_product_penalty_func(point):
	"""
	Returns a multiplicative penalty (assuming higher is good, outputs 0->1)
	based on the dot product of the ball->point vector and our nearest upfield robot -> point vectors
	"""
	width_sixth = constants.Field.Width/6
	if abs(point.x) <= width_sixth:
		return 1
	else:
		return 1 - ((abs(point.x) - width_sixth)/(2*width_sixth))

def chip_completion_probability_penalty_func(point):
	distance = (main.ball().pos - point).mag()
	if distance < constants.OurChipping.MAX_CARRY:
		return 0
	chip_prob = evaluation.passing.eval_chip(main.ball().pos, point, main.system_state().our_robots)
	if distance >= constants.OurChipping.MAX_CAPTURE_DISTANCE: 
		return .5 * chip_prob
	elif distance >= constants.OurChipping.MAX_CARRY and distance < constants.OurChipping.MIN_CAPTURE_DISTANCE: 
		return ((distance - constants.OurChipping.MAX_CARRY)/(constants.OurChipping.MAX_CARRY-constants.OurChipping.MIN_CAPTURE_DISTANCE))**2 * chip_prob
	else: 
		return chip_prob


def get_closest_opponent_distance_or_boundary_to_point(point):
	upfield_rob = get_closest_upfield_opponent_distance_to_point(point) 
	downfield_rob = get_closest_downfield_opponent_distance_to_point(point)
	side_space = constants.Field.Width/2 - abs(point.x)
	up = min(upfield_rob[0], point.y)
	down = min(downfield_rob[0], constants.Field.Length - point.y)
	if side_space== min(side_space,up,down):
		return side_space, None
	return upfield_rob if up < down else downfield_rob

def downfield_space_scoring(point, penalties=[outer_third_linear_penalty_func]):
	penalty_mult = np.prod([p(point) for p in penalties])
	downfield_dist = get_closest_downfield_opponent_distance_to_point(point)
	return penalty_mult * downfield_dist[0]

def opponent_space_scoring(point, penalties=[outer_third_halflinear_penalty_func, chip_completion_probability_penalty_func, evaluation.shooting.eval_shot]):
	penalty_mult = np.prod([p(point) for p in penalties])
	dist = get_closest_opponent_distance_or_boundary_to_point(point)
	#print("PT",point)
	#print("PEN",penalty_mult)
	#print("DD",dist[0])
	#print("TOTAL",penalty_mult * dist[0],'\n')
	return penalty_mult * dist[0]

def downfield_chase_scoring(point, penalties=[outer_third_linear_penalty_func, dot_product_penalty_func]):
	penalty_mult = np.prod([p(point) for p in penalties])
	downfield_dist = get_closest_downfield_opponent_distance_to_point(point)
	#print("PT",point)
	#print("PEN",penalty_mult)
	#print("DD",downfield_dist,'\n')
	return penalty_mult * downfield_dist[0]

def get_best_downfield_space_point(start_point=None, 
									min_radius=1, 
									max_radius=None, 
									radius_resolution=.25, 
									angle_resolution = 25,
									min_upfield_distance=.5, 
									min_downfield_distance=0, 
									pt_scoring_func=opponent_space_scoring):
	if start_point is None:
		start_point = main.ball().pos
	if max_radius is None:
		max_radius = constants.Field.Length - start_point.y

	best_point = None
	best_score = 0
	best_point = None
	for i in range(math.ceil(min_radius/radius_resolution),math.floor(max_radius/radius_resolution)+1):
		points = [pt for pt in get_radius_points(start_point, r=i*radius_resolution, n=50, from_rad=0, to_rad=180) if 
					not constants.Field.TheirGoalZoneShape.contains_point(pt) 
					and constants.Field.FieldRect.contains_point(pt) 
					and get_closest_upfield_opponent_distance_to_point(pt)[0] >= min_upfield_distance 
					and get_closest_downfield_opponent_distance_to_point(pt)[0] >= min_downfield_distance]
		
		try:
			best_r = sorted([ (pt_scoring_func(pt), pt) for pt in points], key=lambda x: x[0], reverse=True)[0]
		except:
			best_r = (0,robocup.Point(1,6))
		
		if best_r[0] > best_score:
			#print(best_r)
			best_score = best_r[0]
			best_point = best_r[1] 
	return best_point