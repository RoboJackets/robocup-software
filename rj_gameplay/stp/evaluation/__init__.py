"""This module contains the interfaces ISituation, IAnalyzer and IPlaySelector."""

from abc import ABC, abstractmethod
from typing import Dict, Tuple, Generic
from enum import IntEnum, auto


import stp.play
import stp.rc as rc
import string


class Situation(enum.Enum):
    NO_SITUATION = auto()
    KICKOFF = auto()
    DEFEND_RESTART_OFFENSIVE = auto()
    DEFEND_RESTART_MIDFIELD = auto()
    DEFEND_RESTART_DEFENSIVE = auto()
    CLEAR = auto()
    MIDFIELD_CLEAR = auto()
    DEFEND_CLEAR = auto()
    MIDFIELD_DEFEND_CLEAR = auto()
    DEFEND_GOAL = auto()
    ATTACK_GOAL = auto()
    OFFENSIVE_SCRAMBLE = auto()
    MIDFIELD_SCRAMBLE = auto()
    DEFENSIVE_SCRAMBLE = auto()
    SAVE_BALL = auto()
    SAVE_SHOT = auto()
    OFFENSIVE_PILEUP = auto()
    MIDFIELD_PILEUP = auto()
    DEFENSIVE_PILEUP = auto()
    SHOOTOUT = auto()
    DEFEND_SHOOTOUT = auto()
    PENALTY = auto()
    DEFEND_PENALTY = auto()
    OFFENSIVE_KICK = auto()
    DEFENSIVE_KICK = auto()
    MIDFIELD_KICK = auto()
    GOALIE_CLEAR = auto()

    descriptions = {
        "NO_SITUATION" : "There is no situation.",
        "KICKOFF" : "We are performing kickoff.",
        "DEFEND_RESTART_OFFENSIVE" : "We are defending a restart in the offensive area of the field",
        "DEFEND_RESTART_MIDFIELD" : "We are defending a restart in the midfield area of the field",
        "DEFEND_RESTART_DEFENSIVE" : "We are defending a restart in the defensive area of the field",
        "CLEAR" : "We have the ball in the defensive area of the field",
        "DEFEND_CLEAR" : "Our opponents have the ball in the offensive area of the field",
        "DEFEND_GOAL" : "Our opponents have the ball in the defensive area of the field",
        "MIDFIELD_CLEAR" : "We have the ball in the midfield are of the field",
        "ATTACK_GOAL" : "We have the ball in the offensive area of the field",
        "OFFENSIVE_SCRAMBLE" : "The ball is unpossessed in the offensive area of the field",
        "MIDFIELD_SCRAMBLE" : "The ball is unpossessed in the midfield area of the field",
        "DEFENSIVE_SCRAMBLE" : "The ball is unpossessed in the defensive area of the field",
        "SAVE_BALL" : "The ball is freely headed towards out of bounds",
        "SAVE_SHOT" : "The ball is headed directly towards our goal",
        "OFFENIVE_PILEUP" : "The ball is wedged between robots of oppisite teams in the offensive area of the field",
        "MIDFIELD_PILEUP" : "The ball is wedged between robots of oppisite teams in the midfield area of the field",
        "DEFENSIVE_PILEUP" : "The ball is wedged between robots of uppisite teams in the defensive area of the field",
        "MIDFIELD_DEFEND_CLEAR" : "Our opponents have the ball in the midfield",
        "SHOOTOUT" : "We are performing a shootout",
        "DEFEND_SHOOTOUT" : "Our opponents are performing a shootout",
        "PENALTY" : "We are performing a penalty kick",
        "DEFEND_PENALTY" : "Our opponents are performing a penalty kick",
        "OFFENSIVE_KICK" : "We are performing a restart in the offensive area of the field",
        "DEFENSIVE_KICK" : "We are performing a restart in the defensive area of the field",
        "MIDFIELD_KICK" : "We are performing a restart in the midfield area of the field",
        "GOALIE_CLEAR" : "Our goalie safely has the ball inside our goalie zone",
        }

    @property
    def description(self) -> str:
        return self.descriptions.get(self.name, default = "")

    @property
    def name_dec(self) -> str:
        return self.fname + self.description

    @property
    def fname(self) -> str:
        return string.capwords(self.name.lower())

    def __str__(self):
        return "Situation: " + self.fname

    def is_clear(self):
        pass

    def is_scramble(self):
        pass

    def is_offensive(self):
        pass

    def is_midfield(self):
        pass

    def is_defensive(self):
        pass


class ISituation(ABC):
    """Interface for a situation."""

    @property
    @abstractclass
    def description(self) -> str: 
        ...

    @property
    @abstractclass
    def name(self) -> str:
        ...

PropT = TypeVar("PropT")

class IEvaluator(Generic[PropT], ABC):

    @abstractmethod
    def compute_props(self, prev_props: Optional[PropT]) -> PropT:
        """Computes the props(state) required for the current tick.
        :param prev_props: The props from the previous tick, if available.
        :return: The props for the current tick.
        """
        ...

    @abstractmethod
    def tick(self, props: PropT) -> None:
        """
        :param props: The state of the current tactic.
        :return: A list of actions to be executed.
        """
        ...


class ISituationAnalyzer(IEvaluator):
    """Interface for situation analyzer."""

    @abstractmethod
    def get_situation(self) -> ISituation:
        ...


class IPlaySelector(ABC):
    """Interface for play selector."""

    @abstractmethod
    def select(self, world_state: rc.WorldState, analyzer: ISituationAnalyzer) -> stp.play.IPlay:
        """Selects the best play given given the current world state.
        :param world_state: The current state of the world.
        :param analyzer: A situation source
        :return: A tuple of the best situation and best play.
        """
        ...
