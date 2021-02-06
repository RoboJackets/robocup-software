import numpy as np
import stp.rc as rc
import stp.testing as testing
import rj_gameplay.gameplay_node as gameplay_node
import rclpy

def test_cost(world_state: rc.WorldState, robot: rc.Robot) -> float:
	print("pain")
	return 1

def main():
	np.ndarray(shape=(3,1))
	world_state = rc.WorldState(["e"], [], None)
	for i in world_state.our_robots:
		test_cost(world_state, i)


if __name__ == "__main__":
	main()