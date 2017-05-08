import behavior_sequence
import single_robot_behavior
import role_assignment

#A sequence to be carried out by a single robot, attempting to add non-single robot behaviors will raise exceptons
class SingleRobotSequence(behavior_sequence.BehaviorSequence, single_robot_behavior.SingleRobotBehavior):
    def __init__(self, behaviors = [], continuos=False, autorestart=lambda: True):
        single_robot_behavior.SingleRobotBehavior.__init__(self, continuos)
        behavior_sequence.BehaviorSequence.__init__(self, behaviors = behaviors)
 
    @property
    def behaviors(self):
        return behavior_sequence.BehaviorSequence.behaviors.fget(self)

    @behaviors.setter
    def behaviors(self, value):
        #Insures that all behaviors are
        for b in value:
            if not isinstance(b, single_robot_behavior.SingleRobotBehavior):
                raise TypeError("SingleRobotSequence requires single robot behaviors")
        self._behaviors = value

    def role_requirements(self):
        #forces all subbhehaviors to use the same robot
        if len(self.behaviors) > 0 and self.robot != None:
            reqs = behavior_sequence.BehaviorSequence.role_requirements(self)
            for req in role_assignment.iterate_role_requirements_tree_leaves(reqs):
                req.required_shell_id = self.robot.shell_id()
        else:
            reqs = single_robot_behavior.SingleRobotBehavior.role_requirements(self)
        return reqs

    def assign_roles(self, assignments):
        oldBot = self.robot
        #Treats the role assignments like a behavior sequence if it has behaviors, otherwise like a single robot behavior
        if (len(self.behaviors) > 0):
            behavior_sequence.BehaviorSequence.assign_roles(self, assignments)
            # extract robot from the one leaf in the tree
            # we don't know how deep the tree is, which is why we use the tree leaf iterator
            for assignment_tuple in role_assignment.iterate_role_requirements_tree_leaves(
                assignments):
                self.robot = assignment_tuple[1]
        else:
            single_robot_behavior.SingleRobotBehavior.assign_roles(self, assignments)
        # Sometimes the robot executing a single_robot_composite_behavior changes in the
        # middle of the behavior. For some plays, this means we should restart the whole
        # behavior for the new robot (autorestart = True). For others, it is more important to continue the new
        # robot where the old robot left off (autorestart = False).
        if oldBot != None and self.robot != None and oldBot.shell_id() != self.robot.shell_id() and self.autorestart():
            logging.info("SingleRobotSequence: robot changed, restarting behavior")
            self.restart()
