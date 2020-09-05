import numpy as np

import stp.rc as rc


# TODO: Add time information.
class Pass:
    """A representation of a pass."""

    __slots__ = ["passer", "receiver", "pt"]

    passer: rc.RobotId
    receiver: rc.RobotId
    pt: np.ndarray

    def __init__(self, passer: rc.RobotId, receiver: rc.RobotId, pt: np.ndarray):
        """Creates a Pass.
        :param passer: The RobotId of the passing robot.
        :param receiver: The RobotId of the receiving robot.
        :param pt: The point at which the pass will be made to.
        """
        self.passer = passer
        self.receiver = receiver
        self.pt = pt
