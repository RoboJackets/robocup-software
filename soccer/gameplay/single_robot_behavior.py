import behavior
import role_assignment
import re
import robocup
from typing import Tuple


class SingleRobotBehavior(behavior.Behavior):
    def __init__(self, continuous):
        super().__init__(continuous)
        self.robot = None

    def execute_running(self):
        if self.robot is None:
            raise AssertionError(
                "Error: execute_running() called on a single robot behavior that doesn't have a robot!")

    def role_requirements(self):
        reqs = role_assignment.RoleRequirements()
        if self.robot is not None:
            reqs.previous_shell_id = self.robot.shell_id()
        return reqs

    # assignments is a (RoleRequirements, OurRobot) tuple

    # Waiting on https://github.com/PyCQA/pylint/issues/1452 to upgrade pylint
    # pylint: disable=invalid-sequence-index
    def assign_roles(self, assignments: Tuple[role_assignment.RoleRequirements,
                                              robocup.OurRobot]):
        if not isinstance(assignments, tuple) or len(assignments) > 2:
            raise AssertionError(
                "Invalid call to assign_roles.  Expected a tuple")
        if len(assignments) == 2:
            if assignments[1] is not None and not isinstance(assignments[1],
                                                             robocup.OurRobot):
                raise TypeError(
                    "ERROR: attempt to assign robot to a non-robot object value: "
                    + str(assignments[1]))
            self.robot = assignments[1]

    def __str__(self):
        desc = super().__str__()
        desc += "[robot=" + (str(self.robot.shell_id())
                             if self.robot is not None else "None") + "]"
        if self.robot is not None:
            indent = '    '
            cmd_text = self.robot.get_cmd_text()[:-1]
            if len(cmd_text) > 0:
                cmd_text = re.sub(r'\n', '\n' + indent, '\n' + cmd_text)
            desc += cmd_text
        return desc
