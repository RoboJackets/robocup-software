from abc import ABC, abstractmethod
import robocup
import force_sample

class Force(ABC):

    @abstractmethod
    def sample(self, sample_point):
        return ForceSample(robocup.Point(0,0), sample_point)

