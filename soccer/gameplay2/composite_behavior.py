import behavior


# a composite behavior is one that has 1+ subbehaviors
# this class has methods for making it easy to work with and manage subbehaviors
class CompositeBehavior(behavior.Behavior):

    def __init__(self, continuous):
        super().__init__(continuous=continuous)

        self._subbehavior_info = {}


    # FIXME: what if a subbehavior of @bhvr is required, but this is not?
    # FIXME: how do priorities work?
    # FIXME: how do nested priorities work?
    def add_subbehavior(self, bhvr, name, required=True, priority=100):
        if name in self._subbehavior_info:
            raise AssertionError("There's already a subbehavior with name: '" + name + "'")
        self._subbehavior_info[name] = {'required': required, 'priority': priority, 'behavior': bhvr}            


    def remove_subbehavior(self, name):
        del self._subbehavior_info[name]


    def subbehavior_with_name(self, name):
        return self._subbehavior_info[name]['behavior']


    def subbehaviors_by_name(self):
        by_name = {}
        for name in self._subbehavior_info:
            by_name[name] = self._subbehavior_info[name]['behavior']
        return by_name


    def all_subbehaviors(self):
        return [self._subbehavior_info[name]['behavior'] for name in self._subbehavior_info]


    def execute_running(self):
        # run each subbehavior
        for name in self._subbehavior_info:
            bhvr = self._subbehavior_info[name]
            if isinstance(bhvr, single_robot_behavior.SingleRobotBehavior):
                if bhvr.robot != None:
                    # only run single robot behaviors when they have a robot
                    bhvr.run()
            else:
                # multi-robot behaviors always get run
                bhvr.run()


    # returns a tree of role_requirements
    def role_requirements(self):
        reqs = {}
        for name, bhvr in self.subbehaviors_by_name().items():
            reqs[name] = bhvr.role_requirements()
        return reqs


    # assignments is a tree with the same structure as that returned by role_requirements()
    # the only difference is that leaf nodes are (RoleRequirements, OurRobot) tuples
    # instead of just RoleRequirements
    def assign_roles(self, assignments):
        for name, subtree in assignments.items():
            self.subbehavior_with_name(name).assign_roles(subtree)


    def __str__(self):
        raise NotImplementedError()
