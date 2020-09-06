from typing import List, Tuple

import numpy as np
import stp.play
import stp.rc as rc
import stp.situation
import stp.situation.analyzer as analyzer
from stp import action as action
from stp.role import assignment as assignment


class SituationA(stp.situation.ISituation):
    def is_applicable(self, world_state: rc.WorldState) -> bool:
        return len(world_state.our_robots) > 0 and world_state.our_robots[0].id == 0

    def score(self, world_state: rc.WorldState) -> float:
        return 0.0


class SituationB(stp.situation.ISituation):
    def is_applicable(self, world_state: rc.WorldState) -> bool:
        return len(world_state.our_robots) > 1 and world_state.our_robots[1].id == 1

    def score(self, world_state: rc.WorldState) -> float:
        return 0.2


class SituationC(stp.situation.ISituation):
    def is_applicable(self, world_state: rc.WorldState) -> bool:
        return len(world_state.our_robots) > 2 and world_state.our_robots[2].id == 2

    def score(self, world_state: rc.WorldState) -> float:
        return 0.5


class BallSituation(stp.situation.ISituation):
    def is_applicable(self, world_state: rc.WorldState) -> bool:
        return any([robot.has_ball for robot in world_state.our_robots])

    def score(self, world_state: rc.WorldState) -> float:
        return 1.0


class BasePlay(stp.play.IPlay):
    def tick(
        self, world_state: rc.WorldState, prev_results: assignment.FlatRoleResults
    ) -> Tuple[assignment.FlatRoleResults, List[action.IAction]]:
        pass


class PlayA(BasePlay):
    ...


class PlayB(BasePlay):
    ...


class PlayC(BasePlay):
    ...


class SituationAnalyzerFixture:
    def __init__(self):
        self.situation_a = SituationA()
        self.situation_b = SituationB()
        self.situation_c = SituationC()
        self.ball_situation = BallSituation()

        self.play_a1 = PlayA()
        self.play_a2 = PlayA()
        self.play_b = PlayB()
        self.play_c = PlayC()

        # Create the play registry.
        self.play_registry: stp.situation.PlayRegistry = {
            self.situation_a: self.play_a1,
            self.situation_b: self.play_b,
            self.situation_c: self.play_c,
            self.ball_situation: self.play_a2,
        }


def test_analyzer_applicable_situations() -> None:
    """Tests that _applicable_situations works correctly."""

    fixture = SituationAnalyzerFixture()

    # Create the SituationAnalyzer.
    situation_analyzer = analyzer.SituationAnalyzer(fixture.play_registry)

    ball = rc.Ball(np.zeros(2), np.zeros(2))

    # With 1 robot with id=0, only SituationA is applicable.
    our_robots = [rc.Robot(0, np.zeros(3), np.zeros(3), False)]
    world_state = rc.WorldState(our_robots, [], ball)

    situations = situation_analyzer._applicable_situations(world_state)

    assert len(situations) == 1
    assert situations[0] == fixture.situation_a

    # With 2 robot with ids [3, 1], only SituationB is applicable.
    our_robots = [
        rc.Robot(robot_id, np.zeros(3), np.zeros(3), False) for robot_id in [3, 1]
    ]
    world_state = rc.WorldState(our_robots, [], ball)

    situations = situation_analyzer._applicable_situations(world_state)

    assert len(situations) == 1
    assert situations[0] == fixture.situation_b

    # With 3 robot with ids [0, 5, 2] and the middle one having the ball, SituationsA,
    # BallSituation SituationC are applicable.
    our_robots = [
        rc.Robot(robot_id, np.zeros(3), np.zeros(3), False) for robot_id in [0, 5, 2]
    ]
    our_robots[1].has_ball = True
    world_state = rc.WorldState(our_robots, [], ball)

    situations = situation_analyzer._applicable_situations(world_state)

    assert len(situations) == 3
    assert fixture.situation_a in situations
    assert fixture.situation_c in situations
    assert fixture.ball_situation in situations


def test_analyzer_select() -> None:
    """Tests that select works correctly."""

    fixture = SituationAnalyzerFixture()

    # Create the SituationAnalyzer.
    situation_analyzer = analyzer.SituationAnalyzer(fixture.play_registry)

    ball = rc.Ball(np.zeros(2), np.zeros(2))

    # With 1 robot with id=0, only SituationA is applicable, thus it and the
    # corresponding play_a1 should be selected.
    our_robots = [rc.Robot(0, np.zeros(3), np.zeros(3), False)]
    world_state = rc.WorldState(our_robots, [], ball)

    situation, play = situation_analyzer.select(world_state)
    assert situation == fixture.situation_a
    assert play == fixture.play_a1

    # With 3 robot with ids [0, 5, 2] and the middle one having the ball, SituationsA,
    # BallSituation SituationC are applicable. Of these, BallSituation has the highest
    # score and so it should be selected.
    our_robots = [
        rc.Robot(robot_id, np.zeros(3), np.zeros(3), False) for robot_id in [0, 5, 2]
    ]
    our_robots[1].has_ball = True
    world_state = rc.WorldState(our_robots, [], ball)

    situation, play = situation_analyzer.select(world_state)
    assert situation == fixture.ball_situation
    assert play == fixture.play_a2
