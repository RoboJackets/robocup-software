"""This module contains Actions of the STP(A) hierarchy."""

from abc import ABC, abstractmethod
from typing import Dict, MutableMapping, Type, TypeVar
import stp.rc as rc
from rj_msgs.msg import RobotIntent
#from rclpy.publisher import Publisher




class IIntentAction(ABC):

    def tick(self, RobotIntent):
        hat


class IAction(ABC):
    """Interface for actions."""

    #@abstractmethod
    def tick(self, RobotIntent) -> None:
        """Ticks the action. Modifying the passed RobotIntent object"""
        ...

    #@abstractmethod
    #def tick(self) -> RobotIntent:
    #    """Ticks the action, returns a RobotIntent to forward the action"""
    #    ...

    #@abstractmethod
    #def tick(self) -> RobotIntent:
    #    """Ticks the action, returns an object with only relavant fields of the RobotIntent message"""
    #    ...
    #"""


class IFiniteAction(IAction, ABC):
    """Represents actions that have a start and a finish
        
       It's likly that this won't be needed once we have a proper action implementation.
    """
    
    @abstractmethod
    def is_done(self, world_state: rc.WorldState) -> bool:
        """
        Function that detects if an action is complete by looking at the world state.
        We don't want to have to do this and we won't once there is bidirectional communication between gameplay and robot-driving systems, which should happen with a proper action implementation.
        """
        ...

    @abstractmethod
    def finish(self) -> None:
        """
        Returns a robot-intent that stops the current function???
        """
        ...


