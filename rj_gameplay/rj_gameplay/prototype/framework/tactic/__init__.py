from abc import ABC, abstractmethod
# import typing
# import msg

class ITactic(ABC):
    """Complex single-robot behavior. (Goalie, Striker, Waller, etc.)"""

    @abstractmethod
    def tick(self, 
            world_state: rc.WorldState, 
            situation: ISituation
            ) -> msg.RobotIntent:
        ...

    @abstractmethod
    def has_passed(self) -> bool:
        ...

    @abstractmethod
    def receive_pass(self) -> bool:
        ...

    @abstractmethod
    def is_done(self, world_state: rc.WorldState) -> bool:
        ...
