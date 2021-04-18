from stp import rc
from rj_gameplay import gameplay_node
import rclpy


"""
get_distance returns the distance between the ball and the player
robot_id: robot's id
"""
def get_distance(world_state: rc.WorldState, robot_id) -> float:
	if world_state is not None:
		robot = world_state.our_robots[robot_id]
		pose_x = robot.pose[0]
		pose_y = robot.pose[1]
		ball_pos_x = world_state.ball.pos[0]
		ball_pos_y = world_state.ball.pos[1]
		distance = ((ball_pos_x-pose_x)**2 + (ball_pos_y-pose_y)**2)**0.5
		return distance
		
"""
get_velocity returns the velocity of the ball
"""
def get_velocity(world_state: rc.WorldState) -> float:
	if world_state is not None:
		ball_vel = world_state.ball.vel
		ball_vel = (ball_vel[0]**2 + ball_vel[1]**2) ** 0.5
		return ball_vel
			

def possession_classifier(world_state: rc.WorldState)-> Union[int, bool]:
	possess_id, team = None, None
	min_dist = 100
	ball_vel = util.get_velocity()

	for robot in world_state.our_robots:
		if robot.has_ball:
			distance = get_distance(gameplay_node.get_world_state(), robot.id)
			if min_dist > distance:
				min_dist = distance
				possess_id = robot.id
				team = True #our team
		else:
			distance = get_distance(gameplay_node.get_world_state(), robot.id)
			if distance < 0.1 and min_dist > distance and ball_vel < 0.05:
				possess_id = robot.id
				min_dist = distance
				team = True

	for robot in world_state.their_robots:
		if robot.has_ball:
			distance = get_distance(gameplay_node.get_world_state(), robot.id)
			if min_dist > distance:
				min_dist = distance
				possess_id = robot.id
				team = False #opponent
		else:
			distance = get_distance(gameplay_node.get_world_state(), robot.id)
			if distance < 0.1 and min_dist > distance and ball_vel < 0.05:
				possess_id = robot.id
				min_dist = distance
				team = False

	return possess_id, team


