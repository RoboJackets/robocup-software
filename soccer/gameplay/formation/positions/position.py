import robocup
import enum

# Parent class of any position
# Keeps valid position data internal

class Position():
    class Type(enum.Enum):
        Striker = 0
        Midfielder = 1
        Defender = 2
        Goalie = 3

    def __init__(self, position_class):
        self.position_class = position_class

        # Where the controller wants this position to be
        # Set by controller in init and should not be touched
        # None when the position is goalie and a relative position
        #  doesn't make sense
        self.relative_pos = None

        # Actual location the controller wants in field XY terms
        # None follows the same rules as the `relative_pos`
        self.target_pos = None

        # List of other positions that we can pass to
        # These are the "triangles" that are naturally formed on the field
        # In general, those in this list are the only ones who need to get
        #  open when I have the ball
        # This is sorted from most "forward" option to furthest "back" option
        #  from left to right in formation
        self.pass_options = []

    # Target location
    # "Neighbors"