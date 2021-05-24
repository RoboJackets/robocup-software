import stp.rc as rc
import rj_gameplay.gameplay_node as gameplay_node
import rclpy
import math
from typing import Union
import util

play_selector = gameplay_node.EmptyPlaySelector()
gameplay_node = gameplay_node.GameplayNode(play_selector)
recipient_id = None
kicker_id = None
kicker_team = None
old_angle = None
new_angle = None

#get_recipient guessing the intended recipient

def get_recipient(world_state: rc.WorldState, team, vel_angle) -> int:
	ball_pos_x = world_state.ball.pos[0]
	ball_pos_y = world_state.ball.pos[1]
	angle = 6.28
	recipient = None
	if team:
		for robot in world_state.our_robots:
			pose_x = robot.pose[0]
			pose_y = robot.pose[1]
			recp_angle = math.atan2(pose_y - ball_pos_y, pose_x - ball_pos_x)
			angle_diff = vel_angle - recp_angle
			if angle_diff > math.pi:
				angle_diff -= math.pi * 2
			if angle_diff < -math.pi:
				angle_diff += math.pi * 2 
			angle_diff = abs(angle_diff)
			if angle_diff < angle:
				recipient = robot.id
				angle = angle_diff
	else:
		for robot in world_state.their_robots:
			pose_x = robot.pose[0]
			pose_y = robot.pose[1]
			recp_angle = math.atan2(pose_y - ball_pos_y, pose_x - ball_pos_x)
			angle_diff = vel_angle - recp_angle
			if angle_diff > math.pi:
				angle_diff -= math.pi * 2
			if angle_diff < -math.pi:
				angle_diff += math.pi * 2 
			angle_diff = abs(angle_diff)
			if angle_diff < angle:
				recipient = robot.id
				angle = angle_diff
	if angle < math.pi / 6:
		return recipient
	else:
		return None

"""
get_velocity_angle returns the ratio of y component of the velocity vector to x component
"""

def get_velocity_angle(world_state: rc.WorldState) -> float:
	if world_state is not None:
		ball_vel = world_state.ball.vel
		vel_angle = math.atan2(ball_vel[1], ball_vel[0])
		return vel_angle

def get_position_vector(world_state: rc.WorldState, recipient_id) -> float:
	if world_state is not None:
		robot = world_state.our_robots[recipient_id]
		vec_x = robot.pose[0] - world_state.ball.pos[0]
		vec_y = robot.pose[1] - world_state.ball.pos[1]
		pos_angle = math.atan2(vec_y, vec_x)
		return pos_angle


def pass_or_fumble(world_state: rc.WorldState, kicker_id, kicker_team, recipient_id, old_angle, new_angle) -> Union[int, float]:
	possess, team = util.possession_classifier(world_state)
	if kicker_id is None:
		kicker_id = possess
		kicker_team = team

	old_angle = new_angle
	new_angle = get_velocity_angle(world_state)
	try:
		if util.get_distance(world_state, kicker_id) > 0.5 and recipient_id is None:
			recipient_id = get_recipient(world_state, kicker_team, new_angle)
			if recipient_id is None:
				print('Fumbled!')
			else:
				print(f'Passed to {recipient_id}')
	except TypeError:
		pass

	#When the velocity angle changes, if the member possesses the ball, passing is succeeded, otherwise intercepted.

	if old_angle is not None and kicker_id is not None and recipient_id is not None and possess is not None:
		if abs(old_angle-new_angle) > 0.1 and possess != kicker_id:
			if recipient_id == possess and team == kicker_team:
				print(f'Successful Pass from #{kicker_id} to #{possess}')
				kicker_id = possess
				recipient_id = None
			elif team == kicker_team: 
				print(f'Successful Pass from #{kicker_id} to #{possess}, not intended one')
				kicker_id = possess
				recipient_id = None
			else:
				print('Pass is blocked!(Fumbled!)')
				recipient_id = None
				kicker_id = possess
				kicker_team = team

	if recipient_id is not None:
		distance = util.get_distance(world_state, recipient_id)
		pos_vec = get_position_vector(world_state, recipient_id)
		angle_diff = pos_vec - new_angle
		if angle_diff > math.pi:
			angle_diff -= math.pi * 2
		if angle_diff < -math.pi:
			angle_diff += math.pi * 2 
		angle_diff = abs(angle_diff)
		if distance < 1 and angle_diff > 1.35:
			print(f'{recipient_id} misssed the ball(Fumbled!)')
			print(f'angle_diff:{angle_diff}, distance:{distance}')
			recipient_id = None
			kicker_id = None
			kicker_team = None
		# if distance < 0.3:
			# print(f'angle_diff:{angle_diff}, distance:{distance}')
		
	return kicker_id, kicker_team, recipient_id, old_angle, new_angle



while (True):
	rclpy.spin_once(gameplay_node)
	world_state = gameplay_node.get_world_state()
	if world_state is not None:
		[kicker_id, kicker_team, recipient_id, old_angle, new_angle] = pass_or_fumble(world_state, kicker_id, kicker_team, recipient_id, old_angle, new_angle)
		

		# print(f'kicker:{kicker_id}, recip:{recipient_id}')
		# angle = get_velocity_angle(world_state)
		# vel = util.get_velocity(world_state)
		# recipient = get_recipient(world_state, True, angle)
		# distance = util.get_distance(world_state, recipient)
		# print(f'angle:{angle}, vel:{vel}, dist:{distance}')



