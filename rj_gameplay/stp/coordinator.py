"""This module contains the implementation of the coordinator."""
import stp.rc as rc


class Coordinator:
    def __init__(self):
        ...

    def tick(self, world_state: rc.WorldState) -> None:
        # Call situational analysis to see which play should be running.
        # Only one play per situation.
        ...
