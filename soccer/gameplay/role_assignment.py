import munkres
import evaluation.double_touch
import robocup


class RoleRequirements:

    def __init__(self):
        self.pos = None
        self.has_ball = False
        self.chipper_preference_weight = 0
        self.required_shell_id = None
        self.previous_shell_id = None
        self.required = False
        self.priority = 0
        self.require_kicking = False


    @property
    def pos(self):
        return self._pos
    @pos.setter
    def pos(self, value):
        if value != None and not isinstance(value, robocup.Point):
            raise TypeError("Unexpected type for pos: " + str(value))
        self._pos = value


    @property
    def has_ball(self):
        return self._has_ball
    @has_ball.setter
    def has_ball(self, value):
        if value != None and not isinstance(value, bool):
            raise TypeError("Unexpected type for has_ball: " + str(value))
        self._has_ball = value


    # rather than having a 'has_chipper' property, we have a property for the weight
    # * a value of float("inf") means that a chipper is required
    # * a value of 0 means we don't care if it has a chipper or not
    # * a value in-between means we want one, but it's not essential
    # This weight is added to the assignment cost for a robot
    @property
    def chipper_preference_weight(self):
        return self._chipper_preference_weight
    @chipper_preference_weight.setter
    def chipper_preference_weight(self, value):
        self._chipper_preference_weight = value


    # if True, requires that the robot has a working ball sensor, a working kicker,
    # and isn't forbidden from touching the ball by the double touch rules
    # Default: False
    @property
    def require_kicking(self):
        return self._require_kicking
    @require_kicking.setter
    def require_kicking(self, value):
        self._require_kicking = value


    @property
    def required_shell_id(self):
        return self._required_shell_id
    @required_shell_id.setter
    def required_shell_id(self, value):
        if value != None and not isinstance(value, int):
            raise TypeError("Unexpected type for required_shell_id: " + str(value))
        self._required_shell_id = value


    @property
    def previous_shell_id(self):
        return self._previous_shell_id
    @previous_shell_id.setter
    def previous_shell_id(self, value):
        if value != None and not isinstance(value, int):
            raise TypeError("Unexpected type for previous_shell_id: " + str(value))
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



# given a role requirements tree (with RoleRequirements or assignment tuples as leaves),
# yields all of the RoleRequiements objects
def iterate_role_requirements_tree_leaves(reqs_tree):
    if isinstance(reqs_tree, RoleRequirements) or isinstance(reqs_tree, tuple):
        yield reqs_tree
    else:
        for subtree in reqs_tree.values():
            yield from iterate_role_requirements_tree_leaves(subtree)



# This error is thrown by assign_roles() when given an impossible assignment scenario
class ImpossibleAssignmentError(RuntimeError): pass



# multiply this by the distance between two points to get the cost
PositionCostMultiplier = 1.0

# how much penalty is there for switching robots mid-play
RobotChangeCost = 1.0

# a default weight for preferring a chipper
# this is tunable
PreferChipper = 5


# uses the munkres/hungarian algorithm to find the optimal role assignments
# works by building a cost matrix for reach robot, role pair, then choosing the assignments to minimize total cost
# If no restraint-satisfying mass assignment exists, throws an ImpossibleAssignmentError
#
# role_reqs is a tree structure containing RoleRequirements
#
# returns a tree with the same structure as @role_reqs, but the leaf nodes have (RoleRequirements, OurRobot) tuples instead of just RoleRequirements objects
def assign_roles(robots, role_reqs):

    # check for empty requrest set
    if len(role_reqs) == 0:
        return {}


    # first we flatten the role_reqs tree structure into a simple list of RoleRequirements
    role_reqs_list = []
    tree_mapping = {}   # holds key paths so we can map the flattened list back into the tree at the end

    # The input and output to assign_roles() are in a tree form that maps to the behavior tree that we're assigning robots for
    # To do our calculations though, we need to have a flat list of RoleRequirements to match to our list of robots
    # The flatten_tree() method does this and keeps track of how to rebuild the tree so we can do so at the end
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


    if len(robots) == 0:
        return {}


    # build the cost matrix
    cost_matrix = []
    for robot in robots:
        cost_row = []
        for req in role_reqs_list:
            cost = 0

            if req.required_shell_id != None and req.required_shell_id != robot.shell_id():
                cost = float("inf")
            elif req.has_ball == True and robot.has_ball() == False:
                cost = float("inf")
            elif req.require_kicking and (robot.shell_id() == evaluation.double_touch.tracker().forbidden_ball_toucher() or not robot.kicker_works() or not robot.ball_sense_works()):
                cost = float("inf")
            else:
                if req.pos != None:
                    cost += PositionCostMultiplier * (req.pos - robot.pos).mag()
                if req.previous_shell_id != None and req.previous_shell_id != robot.shell_id:
                    cost += RobotChangeCost
                if not robot.has_chipper():
                    cost += req.chipper_preference_weight

            cost_row.append(cost)
        cost_matrix.append(cost_row)


    print("role len: " + str(len(role_reqs_list)))
    print("robot len: " + str(len(robots)))
    print("cost_matrix: " + str(cost_matrix))


    # There's a bug in the munkres package that causes it to infinite loop if we give a cost matrix with a row of all infs or a column of all infs
    # Eventually, I'll fix it and submit a fix to them, but for now this'll do
    # 
    # check for rows of all infs
    row_count = len(cost_matrix)
    if row_count > 1:
        for row in cost_matrix:
            if all(val == float("inf") for val in row):
                raise ImpossibleAssignmentError("No assignments possible that satisfy all constraints")
    # if more than one column, check for columns of all infs
    col_count = len(cost_matrix[0])
    if col_count > 1:
        for c in range(len(cost_matrix[0])):
            col = [row[c] for row in cost_matrix]
            if all(val == float("inf") for val in col):
                raise ImpossibleAssignmentError("No assignments possible that satisfy all constraints")

    # # FIXME: priority?????

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
