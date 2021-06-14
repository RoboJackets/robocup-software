import stp.rc as rc
import numpy as np
from typing import Optional
import stp.role as role
from math import atan2, pi


class PassingCost(role.costFn):	
	def __call__(robot: rc.Robot, prev_result: Optional["RoleResult"], world_state: rc.world_state) -> float:
		pass_cost = 0
		angle_threshold = 0.1
		dist_threshold = 3
		punish_amplifier = 1

		our_robots = world_state.our_robots
		their_robots = world_state.their_robots
		recipient = robot
		kicker = [robot for robot in our_robots if robot.has_ball_sense]
		try:
			kicker = kicker[0]
		except:
			print("Our team is not possessing the ball.")
		kicker_recip_angle = math.atan2(recipient.pose[1]-kicker.pose[1], recipient.pose[0]-kicker.pose[0])
		kicker_recip_dist = sqrt((recipient.pose[1]-kicker.pose[1])**2 + (recipient.pose[0]-kicker.pose[0])**2)

		
		for opp_robot in their_robots:
			# Check if there are no opponent robots that can block
			dist_kicker = sqrt((opp_robot.pose[1]-kicker.pose[1])**2 + (opp_robot.pose[0]-kicker.pose[0])**2)
			if dist_kicker <= kicker_recip_dist:
				angle = math.atan2(opp_robot.pose[1]-kicker.pose[1], opp_robot.pose[0]-kicker.pose[0])
				angle_diff = kicker_recip_angle - angle
				if angle_diff > pi:
					angle_diff -= pi*2
				elif angle_diff < pi:
					agnle_diff += pi*2
				angle_diff = abs(angle_diff)
				if angle_diff < angle_threshold:
					pass_cost += 999

			# Make it expensive if recipient is surrounded by opponents
			dist_recipient = sqrt((opp_robot.pose[1]-recipient.pose[1])**2 + (opp_robot.pose[0]-recipient.pose[0])**2)
			if dist_recipient < dist_threshold:
				pass_cost = pass_cost + (dist_threshold-dist_recipient)**2 #amplifying


		# Punish if passing toward downfield
		if recipient.pose[1] < kicker.pose[1]:
			pass_cost = pass_cost + (kicker.pose[1]-recipient.pose[1]) * punish_amplifier


		return pass_cost


