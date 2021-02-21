import stp.rc as rc
import rj_gameplay.gameplay_node as gameplay_node
import rclpy


def fumble_classifier(world_state: rc.WorldState, possess) -> None:
	if possess is None:
		return True
	else: 
		return False


def pass_classifier(world_state: rc.WorldState, possess)-> None:
	if possess is not None:
		return True
	else: 
		return False
	

"""
get_distance returns the distance between the ball and the kicker
num: kicker's id
"""
def get_distance(world_state: rc.WorldState, kicker) -> None:
	if world_state is not None:
		robot = world_state.our_robots[kicker]
		pose_x = robot.pose[0]
		pose_y = robot.pose[1]
		ball_pos_x = world_state.ball.pos[0]
		ball_pos_y = world_state.ball.pos[1]
		distance = ((ball_pos_x-pose_x)**2 + (ball_pos_y-pose_y)**2)**0.5
		return distance

"""
get_velocity_angle returns the ratio of y component of the velocity vector to x component
"""

def get_velocity_angle(world_state: rc.WorldState) -> None:
	if world_state is not None:
		ball_vel = world_state.ball.vel
		vel = (ball_vel[0]**2 + ball_vel[1]**2) ** 0.5
		if ball_vel[0] == 0:
			angle = ball_vel[1] / (ball_vel[0]+0.00001)
		else:
			angle = ball_vel[1] / ball_vel[0]
		return angle

""" ball_possession returns the id of the robot who has the ball"""

def ball_possession(world_state: rc.WorldState) -> None:
	for robot in world_state.our_robots:
		if robot.has_ball:
			num = robot.id
			return num



play_selector = gameplay_node.EmptyPlaySelector()
gameplay_node = gameplay_node.GameplayNode(play_selector)
while (True):
	rclpy.spin_once(gameplay_node)
	possess = ball_possession(gameplay_node.get_world_state())
	try:
		if kicker is None:
			kicker = possess
	except NameError:
		kicker = possess

	try:
		old_angle = new_angle
		new_angle = get_velocity_angle(gameplay_node.get_world_state())
	except NameError:
		new_angle = get_velocity_angle(gameplay_node.get_world_state())

	#When the velocity angle changes, if the member possesses the ball, passing is succeeded, otherwise intercepted.
	try:
		if old_angle is not None and kicker is not None:
			if old_angle != new_angle and possess != kicker:
				passing = pass_classifier(gameplay_node.get_world_state(), possess, kicker)
				fumbling = fumble_classifier(gameplay_node.get_world_state(), possess, kicker)
				if passing:
					kicker = possess
				else: 
					kicker = None
	except NameError:
		pass


	# try:
	# 	distance = get_distance(gameplay_node.get_world_state(), kicker)
	# except NameError:
	# 	pass	
	# 	prev_distance = new_distance
	# 	new_distance = distance
	# 	fumble_classifier(gameplay_node.get_world_state(), new_distance, prev_distance)

