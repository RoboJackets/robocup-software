import main
import robocup
import behavior
import constants

import standard_play
import evaluation
import tactics.coordinated_pass

#Holds a point or a line, with the associted priority(0 is highest priority)
class PointAndPriority:
	def __init__(self, line_pt, priority):
		self.line_pt = line_pt
		self.priority = priority

class BasicCorner(standard_play.StandardPlay):
	def __init__(self):
		super().__init__(continuous = True)
		#the minimum number we want the probability multiplier be
		MINIMUM_MULTIPLIER = 0.5
		refelctor = 1
		#Depending on corner's location the point are refelcted
		if main.ball().pos.x < 0:
			refelctor = -1

		line1 = PointAndPriority(robocup.Line(robocup.Point(-1.9 * refelctor, 7.45), 
			robocup.Point(-1.9 * refelctor, 8.5)), 0)
		line2 = PointAndPriority(robocup.Line(robocup.Point(-0.56 * refelctor, 7.11), 
			robocup.Point(0.44 * refelctor, 6.2)), 1)
		line3 = PointAndPriority(robocup.Line(robocup.Point(2.40 * refelctor, 5.85), 
			robocup.Point(0.99 * refelctor, 5.85)), 2)

		lines = [line1, line2, line3]

		#Takes three points on the line (the two ends and the midpoint) and returns one with highest probability
		def best_point_on_line(PointAndPriority):
			line = PointAndPriority.line_pt
			point1 = line.get_pt(0)
			point2 = line.get_pt(1)
			point3_x = (point1.x + point2.x) * 0.5
			point3_y = (point1.y + point1.y) * 0.5
			point3 = robocup.Point(point3_x, point3_y)
			points = [point1, point2, point3]
			best_point = max(points,key = lambda point: 
				evaluation.passing.eval_pass(main.ball().pos, point))
			return best_point

		best_points = []

		for line in lines:
			best_points.append(PointAndPriority(best_point_on_line(line), line.priority))

		multiplier_increment = MINIMUM_MULTIPLIER/(len(best_points) - 1)

		best_point = max(best_points,key = lambda point: 
				evaluation.passing.eval_pass(main.ball().pos, point.line_pt) 
				- multiplier_increment * point.priority)
		
		self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(best_point.line_pt), 'passing')

