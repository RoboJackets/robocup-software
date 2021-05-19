import numpy as np
import stp.testing as testing
from stp.rc import Ball, Robot, WorldState, Field, GameInfo, GamePeriod, GameRestart, GameState
import warnings


def test_generate_test_robot() -> None:
    bot = testing.generate_test_robot(robot_id=0)
    assert bot.id == 0
    assert bot.is_ours is True
    assert type(bot.pose) is np.ndarray
    assert np.all(bot.pose == np.array([0.0, 0.0, 0.0]))
    assert type(bot.twist) is np.ndarray
    assert np.all(bot.twist == np.array([0.0, 0.0, 0.0]))
    assert bot.visible is True
    assert bot.has_ball_sense is False
    assert bot.kicker_charged is True
    assert bot.kicker_healthy is True
    assert bot.lethal_fault is False

    bot2 = testing.generate_test_robot(robot_id=2,
                                       is_ours=False,
                                       pose=np.array([1.0, 2.0, 3.0]),
                                       twist=np.array([0.0, 2.0, 1.1]),
                                       has_ball_sense=True)

    assert bot2.id == 2
    assert bot2.is_ours is False
    assert np.all(bot2.pose == np.array([1.0, 2.0, 3.0]))
    assert np.all(bot2.twist == np.array([0.0, 2.0, 1.1]))

    #Make warnings into errors
    warnings.filterwarnings('error')
    try:
        assert bot2.has_ball_sense is True
    except RuntimeWarning:
        pass


def test_generate_test_ball() -> None:
    ball = testing.generate_test_ball()
    assert np.all(ball.vel == np.array([0.0, 0.0]))
    assert np.all(ball.pos == np.array([0.0, 0.0]))
    assert ball.visible is True

    warnings.filterwarnings('error')
    invis_ball = testing.generate_test_ball(visible=False)
    try:
        assert np.all(invis_ball.vel == np.array([0.0, 0.0]))
        assert False
    except RuntimeWarning:
        pass

    ball2 = testing.generate_test_ball(pos=np.array([1.0, 2.0]),
                                       vel=np.array([2.0, 3.0]),
                                       visible=True)
    assert np.all(np.array([1.0, 2.0]) == ball2.pos)
    assert np.all(np.array([2.0, 3.0]) == ball2.vel)


def test_generate_test_field() -> None:
    fieldA = testing.generate_divA_field()
    fieldB = testing.generate_divB_field()
    assert fieldB.length_m == 9.0
    assert fieldB.width_m == 6.0
    assert fieldA.length_m == 12.0
    assert fieldA.width_m == 9.0
    assert np.all(fieldA.our_goal_loc == np.array([0.0, 0.0]))
    assert np.all(fieldA.their_goal_loc == np.array([0.0, 12.0]))
    assert np.all(fieldA.center_field_loc == np.array([0.0, 6.0]))
    assert np.all(fieldB.center_field_loc == np.array([0.0, 4.5]))


def test_generate_test_game_info() -> None:
    info = testing.generate_test_playing_gameinfo()
    assert info.period == GamePeriod.FIRST_HALF
    assert info.state == GameState.PLAYING
    assert info.restart == GameRestart.NONE
    warnings.filterwarnings('error')
    try:
        assert info.our_restart == False
        assert False
    except RuntimeWarning:
        pass


def test_generate_test_world_state() -> None:
    worldState = testing.generate_test_worldstate()
    assert len(worldState.our_robots) == 0
    assert len(worldState.their_robots) == 0
    assert len(worldState.robots) == 0
    assert np.all(worldState.ball.pos == np.array([0.0, 0.0]))
    our_bots = list()
    their_bots = list()
    for g in range(1, 12):
        our_bots.append(testing.generate_test_robot(robot_id=g))
        their_bots.append(
            testing.generate_test_robot(robot_id=g, is_ours=False))

    world2 = testing.generate_test_worldstate(
        our_robots=our_bots,
        their_robots=their_bots,
        field=testing.generate_divA_field(),
        ball=testing.generate_test_ball(pos=np.array([0.0, 1.0])))
    assert len(world2.robots) == 22
    assert len(world2.our_robots) == 11
