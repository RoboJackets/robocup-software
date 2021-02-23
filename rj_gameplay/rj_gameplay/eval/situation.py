"""This module contains the decision tree situation analyzer."""

from enum import Enum

import numpy as np

import stp.rc
import time
import stp.evaluation

import rj_gameplay.eval.possession_evaluator as poss_eval
import rj_gameplay.eval.pileup_evaluator as pile_eval
import rj_gameplay.eval.situation_utils

def get_situation(world_state: stp.rc.WorldState, possession_state: poss_eval.PossessionEvaluator.PropsT, pileup_state: pile_eval.PileupEvaluator.PropsT):

    if(world_state.game_info.game_state is rc.GameState.HALT):
        return rc.evaluation.Situation.NO_SITUATION

    field_loc = situation_utils.ballLocation(world_state)
    possession = possession_state.possession
    pileup = pileup_state.is_pileup
    gamestate = 


class FieldLoc(Enum):
    """Enum for representing where the ball is on the field."""

    DEFEND_SIDE = 1
    MIDFIELD = 2
    ATTACK_SIDE = 3


class Possession(Enum):
    """Enum for representing possession."""
    OUR_BALL = 1
    FREE_BALL = 2
    THEIR_BALL = 3


class SituationEvaluator(stp.evaluation.ISituationEvaluator):
    """Class for decision tree based situation analyzer."""

    @dataclass
    class PropT:
        situation: stp.evaluation.Situation = None
        last_situation: stp.evaluation.Situation = None
        situation_change_time: float = None
        fieldloc: FieldLoc = None
        last_fieldloc: FieldLoc = None

        current_pileup: bool = None
        has_ball: dict = None
        has_ball_change_time: dict = None
        has_ball_dutation: dict = None




    @staticmethod
    def tick(world_state: stp.rc.WorldState, prev_props: Optional[PropT]) -> (PropT, dict):
       
    if(prev_props is None):
        props = Props()
    else:
        props = prev_props





    def analyze_situation(
        self, world_state: stp.rc.WorldState, game_info: stp.rc.GameInfo
    ) -> stp.situation.ISituation:
        """Returns the best situation for the current world state based on a hardcoded
        decision tree.
        :param world_state: The current state of the world.
        :param game_info: The information about the state of the game.
        :return: The best situation for the current world state.
        """

        heuristics = HeuristicInformation(world_state, game_info)

        if game_info.is_restart():
            return self.__analyze_restart(world_state, game_info, heuristics)
        else:
            return self.__analyze_normal(world_state, game_info, heuristics)

    @staticmethod
    def __analyze_restart(
        world_state: stp.rc.WorldState,
        game_info: stp.rc.GameInfo,
        heuristics: HeuristicInformation,
    ) -> stp.situation.ISituation:
        if game_info.is_kickoff():
            if game_info.our_restart:
                return dt.plays.Kickoff()
            else:
                return dt.plays.DefendRestartDefensive()

        elif game_info.is_penalty():
            return dt.plays.NoSituation()

        elif game_info.is_direct() or game_info.is_indirect():
            if heuristics.field_loc == FieldLoc.ATTACK_SIDE:
                if game_info.our_restart:
                    return dt.plays.OffensiveKick()
                else:
                    return dt.plays.DefendRestartOffensive()
            elif heuristics.field_loc == FieldLoc.MIDFIELD:
                if game_info.our_restart:
                    return dt.plays.MidfieldKick()
                else:
                    return dt.plays.DefendRestartMidfield()
            elif heuristics.field_loc == FieldLoc.DEFEND_SIDE:
                if game_info.our_restart:
                    return dt.plays.DefensiveKick()
                else:
                    return dt.plays.DefendRestartDefensive()
            else:
                raise RuntimeError("Unknown field_loc {}".format(heuristics.field_loc))

    @staticmethod
    def __analyze_normal(
        world_state: stp.rc.WorldState,
        game_info: stp.rc.GameInfo,
        heuristics: HeuristicInformation,
    ) -> stp.situation.ISituation:
        if heuristics.field_loc == FieldLoc.DEFEND_SIDE:
            if heuristics.is_pileup:
                return dt.plays.DefensivePileup()
            elif heuristics.ball_pos == BallPos.FREE_BALL:
                return dt.plays.DefensiveScramble()
            elif heuristics.field_loc == BallPos.OUR_BALL:
                return dt.plays.Clear()
            elif heuristics.field_loc == BallPos.THEIR_BALL:
                return dt.plays.DefendGoal()
            else:
                raise RuntimeError("Unknown situation")

        elif heuristics.field_loc == FieldLoc.ATTACK_SIDE:
            if heuristics.is_pileup:
                return dt.plays.OffensivePileup()
            elif heuristics.ball_pos == BallPos.FREE_BALL:
                return dt.plays.OffensiveScramble()
            elif heuristics.field_loc == BallPos.OUR_BALL:
                return dt.plays.AttackGoal()
            elif heuristics.field_loc == BallPos.THEIR_BALL:
                return dt.plays.DefendClear()
            else:
                raise RuntimeError("Unknown situation")

        elif heuristics.field_loc == FieldLoc.MIDFIELD_SIDE:
            if heuristics.is_pileup:
                return dt.plays.MidfieldPileup()
            elif heuristics.ball_pos == BallPos.FREE_BALL:
                return dt.plays.MidfieldScramble()
            elif heuristics.field_loc == BallPos.OUR_BALL:
                return dt.plays.MidfieldClear()
            elif heuristics.field_loc == BallPos.THEIR_BALL:
                return dt.plays.MidfieldDefendClear()
            else:
                raise RuntimeError("Unknown situation")

        else:
            raise RuntimeError(
                "Unknown heuristics.field_loc: {}".format(heuristics.field_loc)
            )

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

        field_len: float = game_info.field.length_m
        midfield: float = field_len / 2

        if ball_pos[1] <= midfield:
            return FieldLoc.DEFEND_SIDE
        elif ball_pos[1] > midfield:
            return FieldLoc.ATTACK_SIDE
        else:
            return FieldLoc.MIDFIELD

#    @staticmethod
#    def __calc_ball_pos(
#        world_state: stp.rc.WorldState, game_info: stp.rc.GameInfo
#    ) -> BallPos:
#        """Computes the current ball possession.
#        :param world_state:
#        :param game_info:
#        :return: The current BallPos.
#        """
#
#        # TODO(1578): Re-implement all the logic here.
#
#        return BallPos.FREE_BALL
#
#    @staticmethod
#    def __calc_pileup(
#        world_state: stp.rc.WorldState, game_info: stp.rc.GameInfo
#    ) -> bool:
#        """Computes whether there is a pileup or not.
#        :param world_state:
#        :param game_info:
#        :return: The current BallPos.
#        """
#
#        # TODO(1578): Re-implement all the logic here.
#
#        return False

