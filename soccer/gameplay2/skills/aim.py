import single_robot_behavior
import behavior
import enum


# The Aim skill is used when a robot has the ball to aim at a particular target
# It fails if the ball is fumbled
# This behavior is continuous, meaning that once the aim is 'good', it continues running
# rather than entering the 'completed' state.  To indicate that the aim is good, it enters
# the 'aimed' state, which is a substate of running.  It may change back from 'aimed' to
# 'aiming' if it's parameters change or due to external conditions.
class Aim(single_robot_behavior.SingleRobotBehavior):

    class State(enum.Enum):
        aiming = 1
        aimed = 2


    def __init__(self):
        super().__init__(continuous=True)

        self.add_state(Aim.State.aiming, behavior.Behavior.State.running)
        self.add_state(Aim.State.aimed, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            Aim.State.aiming,
            lambda: True,
            'immediately')

        for state in Aim.State:
            self.add_transition(state,
                behavior.Behavior.State.failed,
                lambda: not self.robot.has_ball(),
                'fumble')

        self.add_transition(Aim.State.aiming,
            Aim.State.aimed,
            lambda: self.is_aimed(),
            'error < threshold and not rotating too fast')

        self.add_transition(Aim.State.aimed,
            Aim.State.aiming,
            lambda: not self.is_aimed(),
            'error > threshold or rotating too fast')


        self.use_windowing = True
        self.target = constants.Field.TheirGoalSegment



    # If target is a Segment and use_windowing is True, it uses the window evaluator to find the best place to aim at on the Segment
    # Default: True
    @property
    def use_windowing(self):
        return self._use_windowing
    @use_windowing.setter
    def use_windowing(self, value):
        self._use_windowing = value


    # The target Segment or Point that we're aiming at
    # Default: the opponent's goal segment
    @property
    def target(self):
        return self._target
    @target.setter
    def target(self, value):
        self._target = value
    
    

    # returns True if we're aimed at our target within our error thresholds and we're not rotating too fast
    def is_aimed(self):
        raise NotImplementedError()


    # we're aiming at a particular point on our target segment, what is this point?
    def aim_target_point(self):
        if isinstance(self.target, robocup.Point):
            return self.target
        elif isinstance(self.target, robocup.Segment):
            if use_windowing:
                # FIXME: what if the parent behavior of Aim wants to set other conditions on the window evaluator such as chipping or excluded bots?
                win_eval = evaluation.window_evaluator.WindowEvaluator()
                windows, best = win_eval.eval_pt_to_seg(self.robot.pos, self.target)
                return best.center()
            else:
                return self.target.center()
        else:
            raise AssertionError("Expected Point or Segment, found: " + str(self.target))


    def execute_aiming(self):
        pt = aim_target_point()
        # TODO: port the functionality from PivotKick
        raise NotImplementedError()


    def execute_aimed(self):
        pt = self.aim_target_point()
        main.system_state().draw_line(robocup.Line(self.robot.pos, pt), constants.Colors.Green, "Aim")
        self.robot.face(pt)


    def execute_running(self):
        # TODO: draw the aim triangle
        pass


    def __str__(self):
        desc = super().__str__()
        # TODO: add error info here
        return desc;

