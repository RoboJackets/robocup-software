# See PEP 0563 : Enabling the future behavior in Python 3.7
# In this current version, it is not allowed to have a return
# type of the current class, which has not fully been defined yet
from __future__ import annotations # type: ignore

import robocup
import single_robot_composite_behavior
import enum
import typing


## Parent class of any position
# Deals with relative positions as well
# as the generic pass options
class Position(single_robot_composite_behavior.SingleRobotCompositeBehavior):
    class Type(enum.Enum):
        Striker = 0
        Midfielder = 1
        Defender = 2
        Goalie = 3

    def __init__(self, position_class: enum.Enum, name: str) -> None:
        super().__init__(continuous=True)

        self._position_class = position_class
        self._str_name = name
        self._relative_pos = None
        self._pass_options = [] # type: typing.List[Position]

        # Actual location the controller wants in field XY terms
        # None follows the same rules as the `relative_pos`
        self.target_pos = None

    ## What type of position this is
    # (Striker/Midfielder/Defender/Goalie etc)
    @property
    def position_class(self) -> enum.Enum:
        return self._position_class

    ## String name of the position (Left/Right/Center etc)
    @property
    def str_name(self) -> str:
        return self._str_name

    ## Where the controller wants this position to be
    # Set by controller in init and should not be touched
    # None when the position is goalie and a relative position
    #  doesn't make sense
    @property
    def relative_pos(self) -> robocup.Point:
        return self._relative_pos

    @relative_pos.setter
    def relative_pos(self, pos):
        self._relative_pos = pos

    ## List of other positions that we can pass to
    # These are the "triangles" that are naturally formed on the field
    # In general, those in this list are the only ones who need to get
    #  open when I have the ball
    # This is sorted from most "forward" option to furthest "back" option
    #  from left to right in formation
    @property
    def pass_options(self) -> typing.List[Position]:
        return self._pass_options

    @pass_options.setter
    def pass_options(self, options: typing.List[Position]):
        self._pass_options = options

    def __str__(self):
        desc = super().__str__()
        desc += "\n    " + self._str_name
        return desc
