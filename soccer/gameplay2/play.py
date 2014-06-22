import composite_behavior


class Play(composite_behavior.CompositeBehavior):

    def __init__(self, continuous):
        super().__init__(continuous)
        self._robots = None


    # TODO: clarify what this does and what the values mean.  is higher or lower more likely to get picked?
    @classmethod
    def score(cls):
        return 0
