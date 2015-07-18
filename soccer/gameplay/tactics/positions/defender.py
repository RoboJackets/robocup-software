import single_robot_behavior
import behavior
import constants
import robocup
import main
from enum import Enum
import math


## The Defender behavior positions a robot on a certain area of the field and defends it
class Defender(single_robot_behavior.SingleRobotBehavior):

	class State(Enum):
		## gets between a particular opponent and the goal.  stays closer to the goal
		marking = 1

		## chilling out in a zone waiting to mark an opponent.  doesn't do this much
		area_marking = 2


	## The area of the field this robot should block
	class Side(Enum):
		left = 1
		center = 2
		right = 3


	def __init__(self, side=Side.center):
		super().__init__(continuous=True)
		self._block_robot = None
		self._area = None
		self._side = side
		self._opponent_avoid_threshold = 2.0
		self._defend_goal_radius = 0.9
		self._win_eval = robocup.WindowEvaluator(main.system_state())

		self._area = robocup.Rect(robocup.Point(-constants.Field.Width/2.0, constants.Field.Length),
			robocup.Point(constants.Field.Width/2.0, 0))
		if self._side is Defender.Side.right:
			self._area.get_pt(0).x = 0
		if self._side is Defender.Side.left:
			self._area.get_pt(1).x = 0

		self.add_state(Defender.State.marking, behavior.Behavior.State.running)
		self.add_state(Defender.State.area_marking, behavior.Behavior.State.running)

		self.add_transition(behavior.Behavior.State.start,
			Defender.State.marking,
			lambda: True,
			"immediately")
		self.add_transition(Defender.State.marking,
			Defender.State.area_marking,
			lambda: not self._area.contains_point(main.ball().pos) and self.block_robot is None,
			"if ball not in area and no robot to block")
		self.add_transition(Defender.State.area_marking, 
			Defender.State.marking,
			lambda: self._area.contains_point(main.ball().pos) or self.find_robot_to_block() is not None,
			"if ball or opponent enters my area")

	def execute_marking(self):
		#main.system_state().draw_line(robocup.Line(self._area.get_pt(0), self._area.get_pt(1)), (127,0,255), "Defender")
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

		# TODO defenders should register themselves with some static list on init
		# TODO make this happen in python-land
		# for (Defender *f :  otherDefenders)
		# {
		# 	if (f->robot)
		# 	{
		# 		_winEval.exclude.push_back(f->robot->pos);
		# 	}
		# }

		windows = self._win_eval.eval_pt_to_seg(target, goal_line)[0]

		best = None
		goalie = main.our_robot_with_id(main.root_play().goalie_id)

		if goalie is not None and self.side is not Defender.Side.center:
			for window in windows:
				if best is None:
					best = window
				elif self.side is Defender.Side.left and window.segment.center.x < goalie.pos.x and window.segment.length > best.segment.length:
					best = window
				elif self.side is Defender.Side.right and window.segment.center.x > goalie.pos.x and window.segment.length > best.segment.length:
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
				self.robot.chip(1)
			else:
				self.robot.kick(1)

	"""
	TODO comment
	"""
	def execute_area_marking(self):
		if self.robot.pos.near_point(robocup.Point(0,0), self._opponent_avoid_threshold):
			self.robot.set_avoid_opponents(False)
		else:
			self.robot.set_avoid_opponents(True)
		goal_target = robocup.Point(0, -constants.Field.GoalDepth/2.0)
		goal_line = robocup.Segment(robocup.Point(-constants.Field.GoalWidth/2.0,0),
			robocup.Point(constants.Field.GoalWidth/2.0,0))

		if self.side is Defender.Side.left:
			goal_line.get_pt(1).x = 0
			goal_line.get_pt(1).y = 0
		if self.side is Defender.Side.right:
			goal_line.get_pt(0).x = 0
			goal_line.get_pt(0).y = 0

		for robot in main.system_state().their_robots:
			self._win_eval.excluded_robots.append(robot)

		if main.root_play().goalie_id is not None:
			self._win_eval.excluded_robots.append(main.our_robot_with_id(main.root_play().goalie_id))

		# TODO (cpp line 186)
		# windows = self._win_eval.
		windows = []

		best = None
		angle = 0.0
		for window in windows:
			if best is None:
				best = window
				angle = window.a0 - window.a1
			elif window.a0 - window.a1 > angle:
				best = window
				angle = window.a0 - window.a1

		shootline = robocup.Segment(robocup.Point(0,0), robocup.Point(0,0))
		if best is not None:
			angle = (best.a0 + best.a1)/2.0
			shootline = robocup.Segment(self._win_eval.origin(), robocup.Point.direction(angle * (math.pi / 180.0))) # FIXME :no origin. 
			main.system_state().draw_line(shootline, (255,0,0), "Defender")

		need_task = False
		if best is not None:
			winseg = best.segment
			arc = robocup.Circle(robocup.Point(0,0), self._defend_goal_radius)
			shot = robocup.Line(shootline.get_pt(0), shootline.get_pt(1))
			dest = [robocup.Point(0,0), robocup.Point(0,0)]
			intersected, dest[0], dest[1] = shot.intersects_circle(arc)
			if intersected:
				self.robot.move_to(dest[0] if dest[0].y > 0 else dest[1])
			else:
				need_task = True
		if need_task:
			self.robot.face(main.ball().pos)

		if main.ball().pos.y < constants.Field.Length / 2.0:
			self.robot.set_dribble_speed(255)

		backVec = robocup.Point(1,0)
		backPos = robocup.Point(-constants.Field.Width / 2, 0)
		shotVec = robocup.Point(main.ball().pos - self.robot.pos)
		backVecRot = robocup.Point(backVec.perp_ccw())
		facing_back_line = ( backVecRot.dot(shotVec) < 0 )
		if not facing_back_line and self.robot.has_ball():
			if self.robot.has_chipper():
				self.robot.chip(1)
			else:
				self.robot.kick(1)

	def find_robot_to_block(self):
		target = None
		for robot in main.system_state().their_robots:
			if robot.visible and self._area.contains_point(robot.pos):
				if target is None or target.pos.dist_to(main.ball().pos) > robot.pos.dist_to(main.ball().pos):
					target = robot
		return target

	@property 
	def block_robot(self):
		return self._block_robot
	@block_robot.setter
	def block_robot(self, value):
		self._block_robot = value

	@property 
	def side(self):
		return self._side
	@side.setter
	def side(self, value):
		self._side = value
		self._area = robocup.Rect(robocup.Point(-constants.Field.Width/2.0, constants.Field.Length),
			robocup.Point(constants.Field.Width/2.0, 0))
		if self._side is Defender.Side.right:
			self._area.get_pt(0).x = 0
		if self._side is Defender.Side.left:
			self._area.get_pt(1).x = 0
