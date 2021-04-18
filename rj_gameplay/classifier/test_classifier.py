import stp.rc as rc
import rj_gameplay.gameplay_node as gameplay_node
import rclpy

def test_classifier(world_state: rc.WorldState, robot: rc.Robot) -> float:

	if world_state is not None:
		print(world_state.ball.pos)
		for their_robot in world_state.their_robots:
			print(cost)
	pass


play_selector = gameplay_node.EmptyPlaySelector()
gameplay_node = gameplay_node.GameplayNode(play_selector)

while (True):
	rclpy.spin_once(gameplay_node)
	test_classifier(gameplay_node.get_world_state())




