import behavior
import role_assignment
import re


class SingleRobotBehavior(behavior.Behavior):

    def __init__(self, continuous):
        super().__init__(continuous)
        self._robot = None


    def execute_running(self):
        if self.robot == None:
            raise AssertionError("Error: execute_running() called on a single robot behavior that doesn't have a robot!")


    @property
    def robot(self):
        return self._robot
    @robot.setter
    def robot(self, value):
        self._robot = value


    def role_requirements(self):
        reqs = role_assignment.RoleRequirements()
        if self.robot != None:
            reqs.previous_shell_id = self.robot.shell_id()
        return reqs


    # assignments is a (RoleRequirements, OurRobot) tuple
    def assign_roles(self, assignments):
        if not isinstance(assignments, tuple) or len(assignments) > 2:
            raise AssertionError("Invalid call to assign_roles.  Expected a tuple")
        if len(assignments) == 2:
            import robocup
            if not isinstance(assignments[1], robocup.OurRobot):
                raise TypeError("ERROR: attempt to assign robot to a non-robot object value: " + str(assignments[1]))
            self.robot = assignments[1]


    def __str__(self):
        desc = super().__str__()
        desc += "[robot=" + (str(self.robot.shell_id()) if self.robot != None else "None") + "]"
        if self.robot != None:
            indent = '    '
            cmd_text = self.robot.get_cmd_text()[:-1]
            if len(cmd_text) > 0:
                cmd_text = re.sub(r'\n', '\n' + indent, '\n' + cmd_text)
            desc += cmd_text
        return desc
