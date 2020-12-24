from stp.gameplay_sub import GameplaySub
import stp.rc as rc

def test_entry_point() -> None:
    """
    a function that print the ball's postion until the ball enters the right half of the field
    """
    WorldState = GameplaySub()
    while True:
        ball = WorldState.get_world_state().ball
        print(ball.pos)
        if ball.pos[0] < 0:
            WorldState.shutdown()
            print('shutdown')
            break

test_entry_point()