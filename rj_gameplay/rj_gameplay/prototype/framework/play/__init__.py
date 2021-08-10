from abc import ABC, abstractmethod
# import typing
# import msg

class IPlay(ABC):
    """Full team control. Assigns a tactic to every robot, every tick.""" 

    @abstractmethod
    def tick(self, 
            world_state: rc.WorldState, 
            situation: ISituation
            ) -> List[msg.RobotIntent]:
        ...

    @abstractmethod
    def is_done(self, world_state: rc.WorldState) -> bool:
        ...
