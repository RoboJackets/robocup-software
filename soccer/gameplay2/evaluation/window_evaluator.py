import robocup
import constants
import main


# A window is a triangle.  WindowEvaluator creates zero or more Windows.
# One vertex is the origin passed to run().
# The side opposite this origin is a part of the original target segment.
class Window:

    def __init__(self, t0, t1):
        self.t0 = t0
        self.t1 = t1


    # a0 and a1 are the angles from the shot-point to @segment

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


    @property
    def t0(self):
        return self._t0
    @t0.setter
    def t0(self, value):
        self._t0 = value

    @property
    def t1(self):
        return self._t1
    @t1.setter
    def t1(self, value):
        self._t1 = value
    

    # the sub-segment of the original target segment that this window represents
    @property
    def segment(self):
        return self._segment
    @segment.setter
    def segment(self, value):
        self._segment = value
    
    


# The window evaluator finds triangles from the given origin point to the given target point/segment
# The eval_ methods return a list of windows and the best window as a tuple
#
# Example usage:
# win_eval = WindowEvaluator()
# windows, best = win_eval.eval_pt_to_seg(kicker.pos, constants.Field.TheirGoalSegment)
#
class WindowEvaluator:

    def __init__(self):
        self.debug = False

        self.chip_enabled = False

        # TODO: these are currently config values on the C++ side
        #       eventually we should tie back into that
        self.chip_max_range = 0.3
        self.chip_min_range = 4.0

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


    # calculate open windows to another robot
    def eval_pt_to_pt(self, origin, target):
        # dir vec is parallel to the target segment
        dir_vec = (target - origin).perp_ccw().normalized()
        segment = robocup.Segment(target + dir_vec * constants.Robot.Radius,
                                    target - dir_vec * constants.Robot.Radius)

        return self.eval_pt_to_seg(origin, segment)


    # calculate open windows into the opponent's goal
    def eval_pt_to_opp_goal(self, origin):
        return self.eval_pt_to_seg(origin, constants.Field.TheirGoalSegment)


    # t0 and t1 are distances from segment.pt[0] along the segment
    # we use these to remove windows and pieces of windows that are blocked
    # modifies @windows in place
    def obstacle_range(self, windows, t0, t1):
        if t0 == t1:
            # ignore degenerate obstacles
            return

        if t0 > t1:
            t0, t1 = t1, t0


        i = 0
        while i != len(windows):
            w = windows[i]

            if t0 <= w.t0 and t1 >= w.t1:
                # this window is fully covered by the obstacle, so remove it
                del windows[i]
            elif t0 > w.t0 and t1 < w.t1:
                # the window fully contains the obstacle, so we split the window
                w.t1 = t0
                w2 = Window(t1, w.t1)
                windows.insert(i+1, w2)
                i += 2
            elif t0 > w.t0 and t0 < w.t1:
                # the obstacle covers the end of the window
                w.t1 = t0
                i += 1
            elif t1 > w.t0 and t1 < w.t1:
                # the obstacle covers the beginning of the window
                w.t0 = t1
                i += 1


    def obstacle_robot(self, windows, origin, target, bot_pos):
        n = (bot_pos - origin).normalized()
        t = n.perp_ccw()
        r = constants.Robot.Radius + constants.Ball.Radius
        seg = robocup.Segment(bot_pos - n*constants.Robot.Radius + t * r,
                                bot_pos - n*constants.Robot.Radius - t * r)

        end = target.delta().magsq()
        extent = [0, end]

        for i in range(2):
            edge = robocup.Line(origin, seg.pt[i])
            d = edge.delta().magsq()

            intersect = edge.line_intersection(target)
            if intersect != None and (intersect - origin).dot(edge.delta()) > d:
                t = (intersect - target.pt[0]).dot(target.delta())
                if t < 0:
                    extent[i] = 0
                elif t > end:
                    extent[i] = end
                else:
                    extent[i] = t
            else:
                # Obstacle has no effect
                return

        self.obstacle_range(windows, extent[0], extent[1])


    def eval_pt_to_seg(self, origin, target):
        end = target.delta().magsq()

        # if target is a zero-length segment, there are no windows
        if end == 0:
            return [], None

        windows = [Window(0, end)]

        # apply the obstacles
        for bot in main.our_robots() + main.their_robots():
            if bot not in self.excluded_robots and bot.visible:
                d = (bot.pos - origin).mag()
                # whether or not we can chip over this bot
                chip_overable = (self.chip_enabled
                                and (d < self.chip_max_range - constants.Robot.Radius)
                                and (d > self.chip_min_range + constants.Robot.Radius))

                if not chip_overable:
                    self.obstacle_robot(windows, origin, target, bot.pos)

        # set the segment and angles for each window
        p0 = target.pt[0]
        delta = target.delta() / end
        for w in windows:
            w.segment = robocup.Segment(p0 + delta * w.t0, p0 + delta * w.t1)
            w.a0 = (w.segment.pt[0] - origin).angle() * constants.RadiansToDegrees
            w.a1 = (w.segment.pt[1] - origin).angle() * constants.RadiansToDegrees

        best = max(windows, key=lambda w: w.segment.delta().magsq())

        # draw the windows if we're in debug mode
        if self.debug:
            for w in windows:
                pts = [origin, w.segment.pt[0], w.segment.pt[1]]
                color = QColor(255, 0, 0) if w == best else QColor(0, 255)
                main.system_state().draw_polygon(pts, 3, color, "Windows")

        return windows, best
