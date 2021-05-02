from stp import rc
from rj_gameplay import gameplay_node
import rclpy
import util


play_selector = gameplay_node.EmptyPlaySelector()
gameplay_node = gameplay_node.GameplayNode(play_selector)
while (True):
	rclpy.spin_once(gameplay_node)
	possess_id, team = util.possession_classifier(gameplay_node.get_world_state())
	print(f'Possessed by #{possess_id}, Team: {team}')
	