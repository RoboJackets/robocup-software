from stp.rc import Ball, Robot, WorldState, Field, GameInfo

def test_Robot() -> None:
    bot = Robot.generate_basic_test_robot(robot_id = 3, is_ours = False)

def test_Ball() -> None:
    ball = Ball.generate_test_ball()

def test_Field() -> None:
    fieldA = Field.generate_divA_field()
    fieldB = Field.generate_divB_field()

def test_GameInfo() -> None:
    info = GameInfo.generate_test_playing_gameinfo() 

def test_WorldState() -> None:
    worldState = WorldState.generate_basic_test_worldstate()
