import single_robot_behavior
import behavior
import constants
import robocup
import evaluation.window_evaluator
import main
from enum import Enum
import math


# The regular defender does a lot of calculations and figures out where it should be
# This defender lets someone else (the Defense tactic) handle calculations and blocks things based on that
class SubmissiveDefender(single_robot_behavior.SingleRobotBehavior):

	class State(Enum):
		marking = 1 		# gets between a particular opponent and the goal.  stays closer to the goal
		# TODO: add clear state to get and kick a free ball


	def __init__(self):
		super().__init__(continuous=True)
		self._block_object = None
		self._opponent_avoid_threshold = 2.0
		self._defend_goal_radius = 0.9

		self.add_state(SubmissiveDefender.State.marking, behavior.Behavior.State.running)

		self.add_transition(behavior.Behavior.State.start,
			SubmissiveDefender.State.marking,
			lambda: True,
			"immediately")


	def execute_marking(self):
		#main.system_state().draw_line(robocup.Line(self._area.get_pt(0), self._area.get_pt(1)), (127,0,255), "SubmissiveDefender")
		self.block_robot = self.find_robot_to_block()
		if self.block_robot is not None:
			# self.robot.add_text("Blocking Robot " + str(self.block_robot.shell_id()), (255,255,255), "RobotText")
			pass
		if self.robot.pos.near_point(robocup.Point(0,0), self._opponent_avoid_threshold):
			self.robot.set_avoid_opponents(False)
		else:
			self.robot.set_avoid_opponents(True)

		target = None
		if self.block_robot is None:
			target = main.ball().pos + main.ball().vel*0.3
		else:
			target = self.block_robot.pos + self.block_robot.vel*0.3

		goal_line = robocup.Segment(robocup.Point(-constants.Field.GoalWidth/2.0,0),
			robocup.Point(constants.Field.GoalWidth/2.0,0))

		self._win_eval.excluded_robots = [self.robot]


		windows = self._win_eval.eval_pt_to_seg(target, goal_line)[0]

		best = None
		goalie = main.our_robot_with_id(main.root_play().goalie_id)

		if goalie is not None and self.side is not SubmissiveDefender.Side.center:
			for window in windows:
				if best is None:
					best = window
				elif self.side is SubmissiveDefender.Side.left and window.segment.center.x < goalie.pos.x and window.segment.length > best.segment.length:
					best = window
				elif self.side is SubmissiveDefender.Side.right and window.segment.center.x > goalie.pos.x and window.segment.length > best.segment.length:
					best = window
		else:
			best_dist = 0
			for window in windows:
				seg = robocup.Segment(window.segment.center(), main.ball().pos)
				new_dist = seg.dist_to(self.robot.pos)
				if best is None or new_dist < best_dist:
					best = window
					best_dist = new_dist

		shoot_seg = None
		if best is not None:
			if self.block_robot is not None:
				dirvec = robocup.Point.direction(self.block_robot.angle * (math.pi/180.0))
				shoot_seg = robocup.Segment(self.block_robot.pos,
					self.block_robot.pos + dirvec*7.0)
			else:
				shoot_seg = robocup.Segment(main.ball().pos, main.ball().pos + main.ball().vel.normalized() * 7.0)

		need_task = False
		if best is not None:
			winseg = best.segment
			if main.ball().vel.magsq() > 0.03 and winseg.segment_intersection(shoot_seg) != None:
				self.robot.move_to(shoot_seg.nearest_point(self.robot.pos))
				self.robot.face_none()
			else:
				winsize = winseg.length()

				if winsize < constants.Ball.Radius:
					need_task = True
				else:
					arc = robocup.Circle(robocup.Point(0,0), self._defend_goal_radius)
					shot = robocup.Line(winseg.center(), target)
					dest = [robocup.Point(0,0), robocup.Point(0,0)]

					intersected, dest[0], dest[1] = shot.intersects_circle(arc)

					if intersected:
						self.robot.move_to(dest[0] if dest[0].y > 0 else dest[1])
						if self.block_robot is not None:
							self.robot.face(self.block_robot.pos)
						else:
							self.robot.face(main.ball().pos)
					else:
						need_task = True
		if need_task:
			self.robot.face(main.ball().pos)


		backVec = robocup.Point(1,0)
		backPos = robocup.Point(-constants.Field.Width / 2, 0)
		shotVec = robocup.Point(main.ball().pos - self.robot.pos)
		backVecRot = robocup.Point(backVec.perp_ccw())
		facing_back_line = ( backVecRot.dot(shotVec) < 0 )
		if not facing_back_line and self.robot.has_ball():
			if self.robot.has_chipper():
				self.robot.chip(255)
			else:
				self.robot.kick(255)


	# the thing we should be blocking
	# could be an OpponentRobot or a Point
	@property 
	def block_object(self):
		return self._block_object
	@block_object.setter
	def block_object(self, value):
		self._block_object = value


	# @property 
	# def side(self):
	# 	return self._side
	# @side.setter
	# def side(self, value):
	# 	self._side = value
	# 	self._area = robocup.Rect(robocup.Point(-constants.Field.Width/2.0, constants.Field.Length),
	# 		robocup.Point(constants.Field.Width/2.0, 0))
	# 	if self._side is SubmissiveDefender.Side.right:
	# 		self._area.get_pt(0).x = 0
	# 	if self._side is SubmissiveDefender.Side.left:
	# 		self._area.get_pt(1).x = 0


	def role_requirements(self):
		reqs = super().role_requirements()
		# FIXME: be smarter
		return reqs
