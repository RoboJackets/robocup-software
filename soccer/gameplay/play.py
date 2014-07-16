import composite_behavior


class Play(composite_behavior.CompositeBehavior):

    def __init__(self, continuous):
        super().__init__(continuous)


    # Return float("inf") if the play cannot be used or a score (lower is better) used to select the best play.
    @classmethod
    def score(cls):
        return 10

    @classmethod
    def is_restart(cls):
    	return False