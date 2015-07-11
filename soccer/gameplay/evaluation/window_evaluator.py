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
        self.max_chip_range = 0.3
        self.min_chip_range = 4.0

        self.excluded_robots = []
        self.hypothetical_robot_locations = []


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
    def min_chip_range(self):
        return self._min_chip_range
    @min_chip_range.setter
    def min_chip_range(self, value):
        self._min_chip_range = value
    
    @property
    def max_chip_range(self):
        return self._max_chip_range
    @max_chip_range.setter
    def max_chip_range(self, value):
        self._max_chip_range = value


    # A list of robots that we'll pretend don't
    # exist when we're doing calculations
    @property
    def excluded_robots(self):
        return self._excluded_robots
    @excluded_robots.setter
    def excluded_robots(self, value):
        self._excluded_robots = value if value != None else []


    # a list of robocup.Point objects
    # the window evaluator adds robot obstacles at these locations
    @property
    def hypothetical_robot_locations(self):
        return self._hypothetical_robot_locations
    @hypothetical_robot_locations.setter
    def hypothetical_robot_locations(self, value):
        self._hypothetical_robot_locations = value if value != None else []
    


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

    def eval_pt_to_our_goal(self, origin):
        return self.eval_pt_to_seg(origin, constants.Field.OurGoalSegment)


    # t0 and t1 are distances from segment.get_pt(0) along the segment
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
                w2 = Window(t1, w.t1)
                w.t1 = t0
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
            else:
                i += 1


    def obstacle_robot(self, windows, origin, target, bot_pos):
        n = (bot_pos - origin).normalized()
        t = n.perp_ccw()
        r = constants.Robot.Radius + constants.Ball.Radius
        seg = robocup.Segment(bot_pos - n*constants.Robot.Radius + t * r,
                                bot_pos - n*constants.Robot.Radius - t * r)

        if self.debug:
            main.system_state().draw_line(seg, constants.Colors.Red, "debug")

        end = target.delta().magsq()
        extent = [0, end]

        for i in range(2):
            edge = robocup.Line(origin, seg.get_pt(i))
            d = edge.delta().magsq()

            intersect = edge.line_intersection(target)
            if intersect != None and (intersect - origin).dot(edge.delta()) > d:
                t = (intersect - target.get_pt(0)).dot(target.delta())
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

        if self.debug:
            main.system_state().draw_line(target, constants.Colors.Blue, "Debug")

        windows = [Window(0, end)]

        # apply the obstacles
        bots = filter(lambda bot: bot not in self.excluded_robots and bot.visible, (list(main.our_robots()) + list(main.their_robots())))
        bot_locations = list(map(lambda bot: bot.pos, bots))
        bot_locations.extend(self.hypothetical_robot_locations)
        for pos in bot_locations:
            d = (pos - origin).mag()
            # whether or not we can chip over this bot
            chip_overable = (self.chip_enabled
                            and (d < self.max_chip_range - constants.Robot.Radius)
                            and (d > self.min_chip_range + constants.Robot.Radius))
            if not chip_overable:
                self.obstacle_robot(windows, origin, target, pos)

        # set the segment and angles for each window
        p0 = target.get_pt(0)
        delta = target.delta() / end

        for w in windows:
            w.segment = robocup.Segment(p0 + delta * w.t0, p0 + delta * w.t1)
            w.a0 = (w.segment.get_pt(0) - origin).angle() * constants.RadiansToDegrees
            w.a1 = (w.segment.get_pt(1) - origin).angle() * constants.RadiansToDegrees

        best = max(windows, key=lambda w: w.segment.delta().magsq()) if len(windows) > 0 else None

        if self.debug and best is not None:
            main.system_state().draw_line(best.segment, constants.Colors.Green, "Debug")
            main.system_state().draw_line(robocup.Line(origin, best.segment.center()), constants.Colors.Green, "Debug")

        # draw the windows if we're in debug mode
        if self.debug:
            for w in windows:
                pts = [origin, w.segment.get_pt(0), w.segment.get_pt(1)]
                color = (255, 0, 0) if w == best else (0, 255, 0)
                main.system_state().draw_polygon(pts, color, "Windows")

        return windows, best
