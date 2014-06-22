import behavior


class SingleRobotBehavior(behavior.Behavior):

    def execute_running(self):
        super().execute_running()
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
            reqs.previous_shell_id = self.robot.shell_id
        return reqs


    # assignments is a (RoleRequirements, OurRobot) tuple
    def assign_roles(self, assignments):
        if not isinstance(assignments, tuple) or len(assignments) > 2:
            raise AssertionError("Invalid call to assign_roles.  Expected a tuple")
        if len(assignments) == 2:
            self.robot = assignments[1]


    def __str__(self):
        return super().__str__() + "[robot=" + str(self.robot.shell_id) if self.robot != None else "None"
