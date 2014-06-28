import robocup


# A window is a triangle.  WindowEvaluator creates zero or more Windows.
# One vertex is the origin passed to run().
# The side opposite this origin is a part of the original target segment.
class Window:
    pass



    # a0 and a1 are the angles from the shot-point

    @property
    def a0(self):
        return self._a0
    @a0.setter
    def a0(self, value):
        self._a0 = value

    @property
    def a1(self):
        return self._a1
    @a1.setter
    def a1(self, value):
        self._a1 = value

    # the sub-segment of the original target segment that this window represents
    @property
    def segment(self):
        return self._segment
    @segment.setter
    def segment(self, value):
        self._segment = value
    
    



class WindowEvaluator:

    def __init__(self):
        self.debug = False

        self.chip_enabled = False

        # FIXME: set chip min/max values
        self.chip_max_range = 3
        self.chip_min_range = 1

        self.excluded_robots = []


    # Defaults to False
    # if True, uses the system state drawing methods to draw Windows
    @property
    def debug(self):
        return self._debug
    @debug.setter
    def debug(self, value):
        self._debug = value
    

    # Defaults to False
    # If enabled, uses min/max range to remove obstacles
    # that can be cleared with chip
    @property
    def chip_enabled(self):
        return self._chip_enabled
    @chip_enabled.setter
    def chip_enabled(self, value):
        self._chip_enabled = value
    
    @property
    def chip_min_range(self):
        return self._chip_min_range
    @chip_min_range.setter
    def chip_min_range(self, value):
        self._chip_min_range = value
    
    @property
    def chip_max_range(self):
        return self._chip_max_range
    @chip_max_range.setter
    def chip_max_range(self, value):
        self._chip_max_range = value


    # A list of robots that we'll pretend don't
    # exist when we're doing calculations
    @property
    def excluded_robots(self):
        return self._excluded_robots
    @excluded_robots.setter
    def excluded_robots(self, value):
        self._excluded_robots = value if value != None else []
