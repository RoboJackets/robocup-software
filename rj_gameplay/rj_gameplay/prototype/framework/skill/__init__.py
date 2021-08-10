from abc import ABC, abstractmethod
# import typing
# import msg

class ISkill(ABC):
    """Atomic single-robot behavior done with a behavior tree. Handles creating ROS2 msgs for GameplayNode to publish.."""

    @abstractmethod
    def tick(self, world_state: rc.WorldState) -> msg.RobotIntent:
        ...

    @abstractmethod
    def is_done(self, world_state: rc.WorldState) -> bool:
        ...
