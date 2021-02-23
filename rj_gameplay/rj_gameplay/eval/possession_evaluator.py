
class Possession(Enum):
    """Enum for representing possession."""
    OUR_BALL = 1
    FREE_BALL = 2
    THEIR_BALL = 3


class PossessionEvaluator(stp.evaluation.IEvaluator):

    @dataclass
    class PropT:
        a = 0

    @staticmethod
    def tick(stp.WorldState):
        a = 0


