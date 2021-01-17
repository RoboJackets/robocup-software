import rclpy
import rj_gameplay.gameplay_node as gameplay_node
import stp.rc as rc

def test_entry_point() -> None:
    """
    a function that print the ball's postion until the ball enters the right half of the field
    """
    play_selector = gameplay_node.EmptyPlaySelector()
    world_state = gameplay_node.GameplayNode(play_selector)
    while True:
        rclpy.spin_once(world_state)
        if world_state.get_world_state() is not None:
            ball = world_state.get_world_state().ball
            print(ball.pos)
            if ball.pos[0] < 0:
                world_state.shutdown()
                print('shutdown')
                break

test_entry_point()