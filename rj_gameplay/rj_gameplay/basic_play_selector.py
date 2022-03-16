from enum import Enum
from typing import Optional, Tuple

import numpy as np
import stp
import stp.rc as rc
import stp.situation as situation

import rj_gameplay.play as plays
import rj_gameplay.situation.decision_tree.analyzer as analyzer
import rj_gameplay.situation.decision_tree.plays as situations
from rj_gameplay.play import (
    basic_defense,
    basic_offense,
    defend_restart,
    defensive_clear,
    kickoff_play,
    penalty_defense,
    penalty_offense,
    prep_penalty_offense,
    restart,
)

POSSESS_MIN_DIST = 0.15
MIN_PASS_SPEED = 0.9
MIN_NEAR_BALL_DIST = 0.35


class BallPos(Enum):
    """Enum for representing the possession of the ball."""

    OUR_BALL = 1
    FREE_BALL = 2
    THEIR_BALL = 3


class FieldLoc(Enum):
    """Enum for representing where the ball is on the field."""

    DEFEND_SIDE = 1
    MIDFIELD = 2
    ATTACK_SIDE = 3


class HeuristicInformation:
    """Class that represents all the heuristic information needed by the decision
    tree situation analyzer.
    """

    __slots__ = ["ball_pos", "field_loc", "is_pileup"]

    ball_pos: BallPos
    field_loc: FieldLoc
    is_pileup: bool

    def __init__(self, world_state: stp.rc.WorldState, game_info: stp.rc.GameInfo):
        self.field_loc = self.__calc_field_loc(world_state, game_info)
        self.ball_pos = self.__calc_ball_pos(world_state, game_info)
        self.is_pileup = self.__calc_pileup(world_state, game_info)

    @staticmethod
    def __calc_field_loc(
        world_state: stp.rc.WorldState, game_info: stp.rc.GameInfo
    ) -> FieldLoc:
        """Computes the current location of the ball.
        :param world_state:
        :param game_info:
        :return: The current FieldLoc.
        """
        ball_pos: np.ndarray = world_state.ball.pos

        field_len: float = world_state.field.length_m
        midfield: float = field_len / 2

        if ball_pos[1] <= midfield:
            return FieldLoc.DEFEND_SIDE
        elif ball_pos[1] > midfield:
            return FieldLoc.ATTACK_SIDE
        else:
            return FieldLoc.MIDFIELD

    @staticmethod
    def __calc_ball_pos(
        world_state: stp.rc.WorldState, game_info: stp.rc.GameInfo
    ) -> BallPos:
        """Computes the current ball possession.
        :param world_state:
        :param game_info:
        :return: The current BallPos.
        """
        for our_bot in world_state.our_robots:
            if (
                our_bot.has_ball_sense
                or np.linalg.norm(
                    np.array(world_state.ball.pos) - np.array(our_bot.pose[0:2])
                )
                < POSSESS_MIN_DIST
            ):
                return BallPos.OUR_BALL
        for their_bot in world_state.their_robots:
            if (
                np.linalg.norm(
                    np.array(world_state.ball.pos) - np.array(their_bot.pose[0:2])
                )
                < POSSESS_MIN_DIST
            ):
                return BallPos.THEIR_BALL
        if np.linalg.norm(world_state.ball.vel) > MIN_PASS_SPEED:
            for our_bot in world_state.our_robots:
                ball_to_bot = np.array(world_state.ball.pos) - np.array(
                    our_bot.pose[0:2]
                )
                ball_to_bot_unit = ball_to_bot / np.linalg.norm(ball_to_bot)
                ball_dir = world_state.ball.vel / np.linalg.norm(world_state.ball.vel)
                dot = abs(np.dot(ball_to_bot_unit, ball_dir))
                if dot > 0.7:
                    return BallPos.OUR_BALL
            for their_bot in world_state.their_robots:
                ball_to_bot = np.array(world_state.ball.pos) - np.array(
                    their_bot.pose[0:2]
                )
                ball_to_bot_unit = ball_to_bot / np.linalg.norm(ball_to_bot)
                ball_dir = world_state.ball.vel / np.linalg.norm(world_state.ball.vel)
                dot = abs(np.dot(ball_to_bot_unit, ball_dir))
                if dot > 0.7:
                    return BallPos.THEIR_BALL
        # TODO(1578): Re-implement actual previous logic here.

        return BallPos.FREE_BALL

    @staticmethod
    def __calc_pileup(
        world_state: stp.rc.WorldState, game_info: stp.rc.GameInfo
    ) -> bool:
        """Computes whether there is a pileup or not.
        :param world_state:
        :param game_info:
        :return: The current BallPos.
        """
        our_near_bots = 0
        their_near_bots = 0
        for their_bot in world_state.their_robots:
            if (
                np.linalg.norm(
                    np.array(world_state.ball.pos) - np.array(their_bot.pose[0:2])
                )
                < MIN_NEAR_BALL_DIST
            ):
                their_near_bots += 1
        for our_bot in world_state.our_robots:
            if (
                np.linalg.norm(
                    np.array(world_state.ball.pos) - np.array(our_bot.pose[0:2])
                )
                < MIN_NEAR_BALL_DIST
            ):
                our_near_bots += 1

        if (
            our_near_bots > 0
            and their_near_bots > 0
            and our_near_bots + their_near_bots >= 3
        ):
            return True
        # TODO(1578): Re-implement all the logic here.

        return False


class BasicPlaySelector(situation.IPlaySelector):
    """Play selector that returns a play based on the situation from analyzer.

    Currently configured to take in only one play per situation. Situation to play mapping is determined in PLAY_DICT constant above. (This means using this class requires importing the whole file, rather than just the class.)
    """

    def __init__(self):
        self.curr_situation = None
        self.curr_play = None

    """    def select(
        self, world_state: rc.WorldState
    ) -> Tuple[Optional[situation.ISituation], stp.play.IPlay]:


        elif analyzer.__calc_ball_pos()
        if world_state.game_info is not None:
            self.curr_situation = self.analyzer.analyze_situation(world_state)


        return (self.curr_situation, self.curr_play)"""

    def select(
        self, world_state: stp.rc.WorldState
    ) -> Tuple[Optional[situation.ISituation], stp.play.IPlay]:
        """Returns the best situation for the current world state based on a hardcoded
        decision tree.
        :param world_state: The current state of the world.
        :param game_info: The information about the state of the game.
        :return: The best situation for the current world state.
        """
        game_info = world_state.game_info
        heuristics = HeuristicInformation(world_state, game_info)

        """if game_info.state == stp.rc.GameState.STOP:
            return dt.plays.Stop()
        elif game_info.is_restart():
            return self.__analyze_restart(world_state, heuristics)"""
        if world_state.game_info is None and self.curr_play is None:
            return (situations.BasicDefense, basic_defense.BasicDefense())
        elif (
            heuristics.ball_pos == BallPos.OUR_BALL
            or heuristics.ball_pos == BallPos.FREE_BALL
        ):
            return (situations.BasicOffense, basic_offense.BasicOffense())
        elif heuristics.ball_pos == BallPos.THEIR_BALL:
            return (situations.BasicDefense, basic_defense.BasicDefense())
