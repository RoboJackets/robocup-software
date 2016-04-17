import single_robot_behavior
import composite_behavior
import role_assignment
import logging
import role_assignment


## Behavior that applies to a single ROBOT and may have up to one subbehavior at any time
# Most of the implementation is just checking if we have subbehaviors and calling the real method on the appropriate superclass
# Note: The method-resolution-order in python dictates that as we have it listed below, SingleRobotBehavior methods take precedence over CompositeBehavior methods
class SingleRobotCompositeBehavior(single_robot_behavior.SingleRobotBehavior,
                                   composite_behavior.CompositeBehavior):

    ## we over-ride this to enforce the rule that there can't be more than one subbehavior
    def add_subbehavior(self, bhvr, name, required=True, priority=100):
        if self.has_subbehaviors():
            raise AssertionError(
                "Attempt to add more than one subbehavior to SingleRobotCompositeBehavior")
        super().add_subbehavior(bhvr, name, required, priority)
        self.robot_shell_id = None

    def has_subbehaviors(self):
        return len(self.all_subbehaviors()) > 0

    def role_requirements(self):
        if self.has_subbehaviors():
            reqs = composite_behavior.CompositeBehavior.role_requirements(self)
            if self.robot != None:
                for req in role_assignment.iterate_role_requirements_tree_leaves(
                        reqs):
                    req.previous_shell_id = self.robot.shell_id()

            return reqs
        else:
            return single_robot_behavior.SingleRobotBehavior.role_requirements(
                self)

    def assign_roles(self, assignments):
        oldBot = self.robot
        if self.has_subbehaviors():
            composite_behavior.CompositeBehavior.assign_roles(self,
                                                              assignments)

            # extract robot from the one leaf in the tree
            # we don't know how deep the tree is, which is why we use the tree leaf iterator
            for assignment_tuple in role_assignment.iterate_role_requirements_tree_leaves(
                    assignments):
                self.robot = assignment_tuple[1]
        else:
            single_robot_behavior.SingleRobotBehavior.assign_roles(self,
                                                                   assignments)

        # Most single robot composite behaviors assume (rightly so) that the robot they're
        # dealing with won't change. PivotKick for example runs the Capture skill, then
        # the Aim skill to do its stuff. It'd be pretty weird if the robot executing Aim
        # was different from the one that just ran Capture. Here we just restart the behavior
        # If it gets assigned a new robot.
        if oldBot != None and self.robot != None and oldBot.shell_id(
        ) != self.robot.shell_id():
            logging.info(
                "SingleRobotCompositeBehavior: robot changed, restarting behavior")
            self.restart()

    def __str__(self):
        if self.has_subbehaviors():
            return composite_behavior.CompositeBehavior.__str__(self)
        else:
            return single_robot_behavior.SingleRobotBehavior.__str__(self)
