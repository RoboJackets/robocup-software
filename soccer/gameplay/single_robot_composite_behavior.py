import single_robot_behavior
import composite_behavior
import role_assignment
import logging
import behavior
from typing import Callable, Union


## Behavior that applies to a single ROBOT and may have up to one subbehavior at any time
# Most of the implementation is just checking if we have subbehaviors and calling the real method on the appropriate superclass
# Note: The method-resolution-order in python dictates that as we have it listed below, SingleRobotBehavior methods take precedence over CompositeBehavior methods
class SingleRobotCompositeBehavior(single_robot_behavior.SingleRobotBehavior,
                                   composite_behavior.CompositeBehavior):
    ## Constructor.
    # If the behavior should not be restarted when it is assigned to a different robot, then autorestart must be false
    # @param continuous should the behavior runs until it is manually stopped
    # @param autorestart function governing the behavior's restarting itself when its robot is switched. Defaults to true
    def __init__(self,
                 continuous=False,
                 autorestart: Callable[[], bool]=lambda: True) -> None:
        single_robot_behavior.SingleRobotBehavior.__init__(
            self, continuous=continuous)
        composite_behavior.CompositeBehavior.__init__(
            self, continuous=continuous)
        self.autorestart = autorestart
        # for pylint
        self.robot = self.robot

    @property
    def autorestart(self) -> Callable[[], bool]:
        return self._autorestart

    @autorestart.setter
    def autorestart(self, value: Callable[[], bool]):
        self._autorestart = value

    ## we over-ride this to enforce the rule that there can't be more
    # than one subbehavior
    def add_subbehavior(self,
                        bhvr: behavior.Behavior,
                        name: str,
                        required: bool=True,
                        priority: Union[int, Callable[[], int]]=100):
        if self.has_subbehaviors():
            raise AssertionError(
                "Attempt to add more than one subbehavior to SingleRobotCompositeBehavior"
            )
        super().add_subbehavior(bhvr, name, required, priority)
        self.robot_shell_id = None

    def has_subbehaviors(self) -> bool:
        return len(self.all_subbehaviors()) > 0

    def role_requirements(self):
        if self.has_subbehaviors():
            reqs = composite_behavior.CompositeBehavior.role_requirements(self)
            if self.robot is not None:
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

        # Sometimes the robot executing a single_robot_composite_behavior changes in the
        # middle of the behavior. For some plays, this means we shouild restart the whole
        # behavior for the new robot (autorestart = True). For others, it is more important to continue the new
        # robot where the old robot left off (autorestart = False).
        if (oldBot is not None and self.robot is not None and
                oldBot.shell_id() != self.robot.shell_id() and
                self.autorestart()):
            logging.info("SingleRobotCompositeBehavior: robot changed (" + str(
                oldBot.shell_id()) + "->" + str(self.robot.shell_id()) +
                         "), restarting: " + type(self).__name__)
            self.restart()

    def __str__(self) -> str:
        if self.has_subbehaviors():
            return composite_behavior.CompositeBehavior.__str__(self)
        else:
            return single_robot_behavior.SingleRobotBehavior.__str__(self)
