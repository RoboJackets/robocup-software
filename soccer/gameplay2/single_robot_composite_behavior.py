import single_robot_behavior
import composite_behavior


# This class is used for behaviors that apply to a single ROBOT and may have up to one subbehavior at any time
# Most of the implementation is just checking if we have subbehaviors and calling the real method on the appropriate superclass
# Note: The method-resolution-order in python dictates that as we have it listed below, SingleRobotBehavior methods take precedence over CompositeBehavior methods
class SingleRobotCompositeBehavior(single_robot_behavior.SingleRobotBehavior, composite_behavior.CompositeBehavior):

    # we over-ride this to enforce the rule that there can't be more than one subbehavior
    def add_subbehavior(self, bhvr, name, required=True, priority=100):
        if self.has_subbehaviors():
            raise AssertionError("Attempt to add more than one subbehavior to SingleRobotCompositeBehavior")
        super().add_subbehavior(bhvr, name, required, priority)


    def has_subbehaviors(self):
        return len(self.all_subbehaviors()) > 0


    def role_requirements(self):
        if self.has_subbehaviors():
            reqs = composite_behavior.CompositeBehavior.role_requirements(self)
            if self.robot != None:
                reqs.previous_shell_id = self.robot.shell_id()
            return reqs
        else:
            return single_robot_behavior.SingleRobotBehavior.role_requirements(self)


    def assign_roles(self, assignments):
        if self.has_subbehaviors():
            composite_behavior.CompositeBehavior.assign_roles(self, assignments)
            self.robot = assignments.values()[0][1]
        else:
            single_robot_behavior.SingleRobotBehavior.assign_roles(self, assignments)


    def __str__(self):
        if self.has_subbehaviors():
            return composite_behavior.CompositeBehavior.__str__(self)
        else:
            return single_robot_behavior.SingleRobotBehavior.__str__(self)
