import single_robot_behavior
import robocup
import evaluation.window_evaluator
import constants
import main
import role_assignment

## this is the abstract superclass for PivotKick and LineKick
class _Kick(single_robot_behavior.SingleRobotBehavior):

    def __init__(self):
        super().__init__(continuous=False)

        self.enable_kick = True
        self.use_chipper = False
        self.kick_power = 100.0
        self.chip_power = constants.Robot.Chipper.MaxPower

        self.use_windowing = True
        self.win_eval_params = {}
        self.target = constants.Field.TheirGoalSegment

        # cached calculated values
        self._aim_target_point = None # this is what our calculations on the given target boil down to

        self.shot_obstacle_ignoring_robots = []


    ## if True, uses the window evaluator to choose the best place to aim at target_segment
    # Default: True
    @property
    def use_windowing(self):
        return self._use_windowing
    @use_windowing.setter
    def use_windowing(self, value):
        self._use_windowing = value


    ## these params are passed to the window evaluator using setattr()
    # Default: {}
    @property
    def win_eval_params(self):
        return self._win_eval_params
    @win_eval_params.setter
    def win_eval_params(self, value):
        self._win_eval_params = value
    


    ## The thing we're trying to kick at
    # can be a Segment or a Point
    # setting this property automatically recalculates the target_aim_point
    # Default: the opponent's goal segment
    @property
    def target(self):
        return self._target
    @target.setter
    def target(self, value):
        self._target = value
        self.recalculate_aim_target_point()


    ## A list of robots that the shot obstacle doesn't apply to
    # Note: the shot obstacle already doesn't apply to the kicker, you don't have to specify that here
    # Default: []
    @property
    def shot_obstacle_ignoring_robots(self):
        return self._shot_obstacle_ignoring_robots
    @shot_obstacle_ignoring_robots.setter
    def shot_obstacle_ignoring_robots(self, value):
        if value == None:
            value = []
        self._shot_obstacle_ignoring_robots = value


    ## We calculate the point we're ACTUALLY going to aim at based on the target Segment/Point and other parameters
    # This is that point
    @property
    def aim_target_point(self):
        return self._aim_target_point


    ## we're aiming at a particular point on our target segment, what is this point?
    def recalculate_aim_target_point(self):
        if self.robot != None:
            # find the point we want to aim at
            if isinstance(self.target, robocup.Point):
                self._aim_target_point = self.target
            elif isinstance(self.target, robocup.Segment):
                if self.use_windowing:
                    win_eval = evaluation.window_evaluator.WindowEvaluator()
                    for key, value in self.win_eval_params.items():
                        setattr(win_eval, key, value)
                    win_eval.chip_enabled = self.robot.has_chipper() and self.use_chipper
                    windows, best = win_eval.eval_pt_to_seg(main.ball().pos, self.target)
                    if best != None:
                        self._aim_target_point = best.segment.center()
                    else:
                        self._aim_target_point = self.target.center()
                else:
                    self._aim_target_point = self.target.center()
            else:
                raise AssertionError("Expected Point or Segment, found: " + str(self.target))


    ## Allows for different kicker/chipper settings, such as for
    # passing with lower power.
    # Default: full power
    @property
    def kick_power(self):
        return self._kick_power
    @kick_power.setter
    def kick_power(self, value):
        self._kick_power = int(value)
    @property
    def chip_power(self):
        return self._chip_power
    @chip_power.setter
    def chip_power(self, value):
        self._chip_power = int(value)


    ## If false, uses straight kicker, if true, uses chipper (if available)
    # Default: False
    @property
    def use_chipper(self):
        return self._use_chipper
    @use_chipper.setter
    def use_chipper(self, value):
        self._use_chipper = value


    ## If set to False, will get all ready to go, but won't kick/chip just yet
    # Can be used to synchronize between behaviors
    # Default: True
    @property
    def enable_kick(self):
        return self._enable_kick
    @enable_kick.setter
    def enable_kick(self, value):
        self._enable_kick = value


    ## creates a polygon obstacle from the ball to the target
    # this obstacle applies to all robots except the kicker and anything specified in the excluded_robots parameter
    # NOTE: this method is not called by _kick, it's up to subclasses/superbehaviors to call it
    def add_shot_obstacle(self, excluded_robots=[]):
        pt = self.aim_target_point
        if pt != None:
            # segment centered at the target point that's @width wide and perpendicular to the shot
            shot_perp = (main.ball().pos - pt).perp_ccw().normalized()
            width = 0.2
            a = pt + shot_perp * width / 2.0
            b = pt - shot_perp * width / 2.0

            # build the obstacle polygon
            obs = robocup.Polygon()
            obs.add_vertex(main.ball().pos)
            obs.add_vertex(a)
            obs.add_vertex(b)

            # tell the bots to not go there
            for bot in main.our_robots():
                if bot not in excluded_robots + [self.robot]:
                    bot.add_local_obstacle(obs)


    def execute_running(self):
        self.add_shot_obstacle(self.shot_obstacle_ignoring_robots)
