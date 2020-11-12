
from abc import ABC
from rj_msgs import WorldState

##
# A base class for evaluation code that needs to be updated each frame.
#
class Evaluator(ABC):

    ##
    # The tick method will be called each frame with the new WorldState and 
    # should update the internal state of the evaluator.
    @classmethod
    @abstractmethod
    def tick(cls, world: WorldState): -> None
        pass

