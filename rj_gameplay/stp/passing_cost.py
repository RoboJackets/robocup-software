import stp.rc as rc
import numpy as np
from typing import Optional
import stp.role as role
from math import atan2, pi
import numpy as np

class PassingCost(role.costFn):	
	def __call__(robot: rc.Robot, prev_result: Optional["RoleResult"], world_state: rc.WorldState) -> float:
		pass_cost = 0
		angle_threshold = 5
		dist_threshold = 3
		punish_amplifier = 1

		our_robots = world_state.our_robots
		opp_robots = world_state.their_robots
		recipient = robot
		kicker = [robot for robot in our_robots if robot.has_ball_sense]
		try:
			kicker = kicker[0]
		except:
			return 9999.

		kicker_loc = kicker.pose[0:2]
		recip_loc = recipient.pose[0:2]
		v_kick_recip = recip_loc - kicker_loc
		kick_recip_dist = np.linalg.norm(recip_loc - kicker_loc)

		
		for opp_robot in opp_robots:
			# Check if there are no opponent robots that can block
			opp_robot_loc = opp_robot.pose[0:2]
			kick_opp_dist = np.linalg.norm(opp_robot_loc - kicker_loc)

			if kick_opp_dist <= kick_recip_dist:
				v_kick_op = opp_robot_loc - kicker_loc
				angle = np.degrees(abs(atan2(np.linalg.det([v_kick_recip, v_kick_op]), np.dot(v_kick_recip, v_kick_op))))

				if angle < angle_threshold:
					pass_cost += 999

			# Make it expensive if recipient is surrounded by opponents
			recip_opp_dist = np.linalg.norm(opp_robot_loc - recip_loc)
			if recip_opp_dist < dist_threshold:
				pass_cost = pass_cost + (dist_threshold-recip_opp_dist)**2 #amplifying


		# Punish if passing toward downfield
		if recipient.pose[1] < kicker.pose[1]:
			pass_cost = pass_cost + (kicker.pose[1]-recipient.pose[1]) * punish_amplifier

		return pass_cost

