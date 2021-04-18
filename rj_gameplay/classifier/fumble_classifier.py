import stp.rc as rc
import rj_gameplay.gameplay_node as gameplay_node
import rclpy
from math import atan2
from typing import Union
import util



#get_recipient guessing the intended recipient

def get_recipient(world_state: rc.WorldState, team, vel_angle) -> int:
	ball_pos_x = world_state.ball.pos[0]
	ball_pos_y = world_state.ball.pos[1]
	angle = 6.28
	if team:
		for robot in world_state.our_robots:
			pose_x = robot.pose[0]
			pose_y = robot.pose[1]
			recp_angle = atan2(pose_y - ball_pos_y, pose_x - ball_pos_x)
			angle_diff = abs(vel_angle - recp_angle)
			if angle_diff < angle:
				recipient = robot.id
				angle = angle_diff
	else:
		for robot in world_state.their_robots:
			pose_x = robot.pose[0]
			pose_y = robot.pose[1]
			recp_angle = atan2(pose_y - ball_pos_y, pose_x - ball_pos_x)
			angle_diff = abs(vel_angle - recp_angle)
			if angle_diff < angle:
				recipient = robot.id
				angle = angle_diff
	return recipient

"""
get_velocity_angle returns the ratio of y component of the velocity vector to x component
"""

def get_velocity_angle(world_state: rc.WorldState) -> float:
	if world_state is not None:
		ball_vel = world_state.ball.vel
		vel_angle = atan2(ball_vel[1], ball_vel[0])
		return vel_angle



def pass_or_fumble(world_state: rc.WorldState, kicker_id, old_angle, new_angle) -> Union[int, float]:
	possess, team = util.possession_classifier(world_state)
	if kicker_id is None:
		kicker_id = possess
		kicker_team = team

	old_angle = new_angle
	new_angle = get_velocity_angle(world_state)
	if possess is None and recipient_id is None:
		recipient_id = get_recipient(world_state, team, new_angle)

	if recipient_id is not None:
		try:
			old_distance = new_distance 
			new_distance = util.get_distance(world_state, recipient_id)
			if old_distance < new_distance:
				print('Fumbled')
		except:
			new_distance = util.get_distance(world_state, recipient_id)
		
	#When the velocity angle changes, if the member possesses the ball, passing is succeeded, otherwise intercepted.

	if old_angle is not None and kicker_id is not None:
		if abs(old_angle-new_angle) > 0.001 and possess != kicker_id:
			# fumbling = fumble_classifier(world_state, possess)
			if recipient_id == possess and team == kicker_team:
				print(f'Successful Pass from #{kicker_id} to #{possess}')
				kicker_id = possess
			else: 
				print('Fumbled!')
				kicker_id = None
				kicker_team = None

	return kicker_id, old_angle, new_angle


play_selector = gameplay_node.EmptyPlaySelector()
gameplay_node = gameplay_node.GameplayNode(play_selector)
recipient_id = None
kicker_id = None
kicker_team = None
old_angle = None
new_angle = None

while (True):
	rclpy.spin_once(gameplay_node)
	[kicker_id, old_angle, new_angle] = pass_or_fumble(gameplay_node.get_world_state(), kicker_id, old_angle, new_angle)




# def pass_fumble(world_state: rc.WorldState, kicker  ):
# 	possess = ball_possession(world_state)
# 	try:
# 		if kicker is None:
# 			kicker = possess
# 	except NameError:
# 		kicker = possess

# 	try:
# 		old_angle = new_angle
# 		new_angle = get_velocity_angle(world_state)
# 	except NameError:
# 		new_angle = get_velocity_angle(world_state)

# 	#When the velocity angle changes, if the member possesses the ball, passing is succeeded, otherwise intercepted.
# 	try:
# 		if old_angle is not None and kicker is not None:
# 			if old_angle != new_angle and possess != kicker:
# 				passing = pass_classifier(world_state, possess)
# 				fumbling = fumble_classifier(gameplay_node.get_world_state(), possess)
# 				if passing:
# 					kicker = possess
# 				else: 
# 					kicker = None
# 	except NameError:
# 		pass
# 	return 

	# try:
	# 	distance = get_distance(gameplay_node.get_world_state(), kicker)
	# except NameError:
	# 	pass	
	# 	prev_distance = new_distance
	# 	new_distance = distance
	# 	fumble_classifier(gameplay_node.get_world_state(), new_distance, prev_distance)

