import munkres


class RoleRequirements:

    def __init__(self):
        self.pos = None
        self.has_ball = False
        self.has_chipper = False
        self.required_shell_id = None
        self.previous_shell_id = None
        self.required = False
        self.priority = 0


    @property
    def pos(self):
        return self._pos
    @pos.setter
    def pos(self, value):
        self._pos = value


    @property
    def has_ball(self):
        return self._has_ball
    @has_ball.setter
    def has_ball(self, value):
        self._has_ball = value


    @property
    def has_chipper(self):
        return self._has_chipper
    @has_chipper.setter
    def has_chipper(self, value):
        self._has_chipper = value


    @property
    def required_shell_id(self):
        return self._required_shell_id
    @required_shell_id.setter
    def required_shell_id(self, value):
        self._required_shell_id = value


    @property
    def previous_shell_id(self):
        return self._previous_shell_id
    @previous_shell_id.setter
    def previous_shell_id(self, value):
        self._previous_shell_id = value


    # Whether or not this Role is required
    # If false, it is assigned
    @property
    def required(self):
        return self._required
    @required.setter
    def required(self, value):
        self._required = value


    @property
    def priority(self):
        return self._priority
    @priority.setter
    def priority(self, value):
        self._priority = value



# This error is thrown by assign_roles() when given an impossible assignment scenario
class ImpossibleAssignmentError(RuntimeError): pass



# multiply this by the distance between two points to get the cost
PositionCostMultiplier = 1.0

# how much penalty is there for switching robots mid-play
RobotChangeCost = 1.0


# uses the munkres/hungarian algorithm to find the optimal role assignments
# works by building a cost matrix for reach robot, role pair, then choosing the assignments to minimize total cost
# If no restraint-satisfying mass assignment exists, throws an ImpossibleAssignmentError
#
# role_reqs is a tree structure containing RoleRequirements
#
# returns a tree with the same structure as @role_reqs, but the leaf nodes have (RoleRequirements, OurRobot) tuples instead of just RoleRequirements objects
def assign_roles(robots, role_reqs):

    # first we flatten the role_reqs tree structure into a simple list of RoleRequirements
    role_reqs_list = []
    tree_mapping = {}   # holds key paths so we can map the flattened list back into the tree at the end

    def flatten_tree(tree, path_prefix=[]):
        for key, subtree in tree.items():
            if isinstance(subtree, dict):
                flatten_tree(subtree, path_prefix + [key])
            elif isinstance(subtree, RoleRequirements):
                role_reqs_list.append(subtree)
                tree_mapping[subtree] = path_prefix + [key]
            else:
                raise AssertionError("Unknown node type in role_reqs tree: " + str(subtree))

    flatten_tree(role_reqs)


    required_roles = [r for r in role_reqs_list if r.required]
    optional_roles = sorted([r for r in role_reqs_list if not r.required], reverse=True, key=lambda r: r.priority)

    # make sure there's enough robots
    if len(required_roles) > len(robots):
        raise ImpossibleAssignmentError("More required roles than available robots")
    elif len(role_reqs_list) > len(robots):
        # remove the lowest priority optional roles so we have as many bots as roles we're trying to fill
        overflow = len(role_reqs_list) - len(robots)
        role_reqs_list = required_roles + optional_roles[0:-overflow]


    # build the cost matrix
    cost_matrix = []
    for robot in robots:
        cost_row = []
        for req in role_reqs_list:
            cost = 0

            if req.required_shell_id != None and req.required_shell_id != robot.shell_id:
                cost = float("inf")
            elif req.has_chipper == True and robot.has_chipper == False:
                cost = float("inf")
            elif req.has_ball == True and robot.has_ball() == False:
                cost = float("inf")
            else:
                if req.pos != None:
                    cost += PositionCostMultiplier * (req.pos - robot.pos).mag()
                if req.previous_shell_id != None and req.previous_shell_id != robot.shell_id:
                    cost += RobotChangeCost

            cost_row.append(cost)
        cost_matrix.append(cost_row)


    # FIXME: priority?????

    # solve
    solver = munkres.Munkres()
    indexes = solver.compute(cost_matrix)


    results = {}

    # build assignments mapping
    assignments = {}
    total = 0
    for row, col in indexes:
        total += cost_matrix[row][col]

        bot = robots[row]
        reqs = role_reqs_list[col]

        # get the keypath of this entry so we can insert back into the tree
        tree_path = tree_mapping[reqs]

        parent = results
        for key in tree_path[:-1]:
            if key not in parent:
                parent[key] = {}
            parent = parent[key]
        parent[tree_path[-1]] = (reqs, bot)


    if total == float("inf"):
        raise ImpossibleAssignmentError("No assignments possible that satisfy all constraints")

    return results
