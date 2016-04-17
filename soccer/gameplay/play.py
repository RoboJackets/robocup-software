import composite_behavior


## @brief A play coordinates the entire team of robots
# @details Only play runs at a time.
# By default, the RootPlay (which every Play is a subbehavior of) handles the Goalie,
# however, by overriding the handles_goalie() class method, a Play can choose to handle
# the goalie on its own, which allows for greater coordination.
class Play(composite_behavior.CompositeBehavior):
    def __init__(self, continuous):
        super().__init__(continuous)

    ## Used to determine when to run a play
    # Return float("inf") if the play cannot be used or a score (lower is better) used to select the best play.
    @classmethod
    def score(cls):
        return 10

    @classmethod
    def is_restart(cls):
        return False

    ## Override to opt-in to handling the Goalie
    # By default, the root play allocates and runs the goalie behavior
    # and the play handles the rest of the bots.
    # However, it is often better to let the play handle it so the goalie can coordinate better
    # with the other bots on the field.
    # In your play, override this method to return True to do the goalie-handling yourself
    # Your play will read the goalie_id from root_play and setup some sort of goalie-ish behavior
    # that that bot will fulfill
    # Note: keep in mind that the global goalie_id can be None and it may change from time to time
    @classmethod
    def handles_goalie(cls):
        return False
