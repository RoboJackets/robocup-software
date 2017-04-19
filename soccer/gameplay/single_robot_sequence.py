import behavior_sequence
import single_robot_behavior
import role_assignment

#A sequence to be carried out by a single robot, attempting to add non-single robot behaviors will raise exceptons
class SingleRobotSequence(behavior_sequence.BehaviorSequence):
    def __init__(self, behaviors = [], autorestart=lambda: True):
        super().__init__()
        self.behaviors = behaviors
        self.autorestart = autorestart
        self._robot = None


    @property
    def behaviors(self):
        return self._behaviors

    @behaviors.setter
    def behaviors(self, value):
        for b in value:
            if not isinstance(b, single_robot_behavior.SingleRobotBehavior):
                raise TypeError("SingleRobotSequence requires single robot behaviors")
        self._behaviors = value

    @property
    def robot(self):
        return self._robot

    @robot.setter
    def robot(self, value):
        self._robot = value

    @property
    def autorestart(self):
        return self._autorestart

    @autorestart.setter
    def autorestart(self, value):
        self._autorestart = value


    def role_requirements(self):
        reqs = super().role_requirements()
        if len(self.behaviors) > 0 and self.robot != None:
                for req in role_assignment.iterate_role_requirements_tree_leaves(reqs):
                    req.required_shell_id = self.robot.shell_id()
        return reqs

    def assign_roles(self, assignments):
        oldBot = self.robot
        super().assign_roles(assignments)
        if (len(self.behaviors) > 0):
            # extract robot from the one leaf in the tree
            # we don't know how deep the tree is, which is why we use the tree leaf iterator
            for assignment_tuple in role_assignment.iterate_role_requirements_tree_leaves(
                assignments):
                self.robot = assignment_tuple[1]

        # Sometimes the robot executing a single_robot_composite_behavior changes in the
        # middle of the behavior. For some plays, this means we should restart the whole
        # behavior for the new robot (autorestart = True). For others, it is more important to continue the new
        # robot where the old robot left off (autorestart = False).
        if oldBot != None and self.robot != None and oldBot.shell_id() != self.robot.shell_id() and self.autorestart():
            logging.info("SingleRobotSequence: robot changed, restarting behavior")
            self.restart()
