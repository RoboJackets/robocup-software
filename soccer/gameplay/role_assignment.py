import munkres
import evaluation.double_touch
import robocup
import logging
import math

# TODO arbitrary cost lambda property


class RoleRequirements:
    def __init__(self):
        self.destination_shape = None
        self.has_ball = False
        self.chipper_preference_weight = 0
        self.required_shell_id = None
        self.previous_shell_id = None
        self.required = False
        self.priority = 0
        self.require_kicking = False
        self.robot_change_cost = 1.0

        # multiply this by the distance between two points to get the cost
        self.position_cost_multiplier = 1.0
        
        # A lambda function property that allows customization of cost
        # Has exactly one parameter, which is a robot
        self.cost_func = lambda r: 0

    def __str__(self):
        props = []
        props.append("has_ball=" + str(self.has_ball))
        if self.destination_shape != None:
            props.append("pos=" + str(self.destination_shape))
        props.append("chip_pref=" + str(self.chipper_preference_weight))
        if self.required_shell_id != None:
            props.append("required_id=" + str(self.required_shell_id))
        if self.previous_shell_id != None:
            props.append("prev_id=" + str(self.previous_shell_id))
        props.append("required=" + str(self.required))
        props.append("can_kick=" + str(self.require_kicking))

        return "; ".join(props)

    def __repr__(self):
        return str(self)

    ## A generic object for determining cost from a point
    # This can be either a robocup.Point or a robocup.Segment
    # The distance from this point will be evaluated linearly into the cost function
    # for role assignments.
    @property
    def destination_shape(self):
        return self._destination_shape

    @destination_shape.setter
    def destination_shape(self, value):
        if value != None and not (isinstance(value, robocup.Point) or
                                  isinstance(value, robocup.Segment)):
            raise TypeError("Unexpected type for destination_shape: " + str(
                value))
        self._destination_shape = value

    @property
    def position_cost_multiplier(self):
        return self._position_cost_multiplier

    @position_cost_multiplier.setter
    def position_cost_multiplier(self, value):
        self._position_cost_multiplier = value

    @property
    def has_ball(self):
        return self._has_ball

    @has_ball.setter
    def has_ball(self, value):
        if value != None and not isinstance(value, bool):
            raise TypeError("Unexpected type for has_ball: " + str(value))
        self._has_ball = value

    # rather than having a 'has_chipper' property, we have a property for the weight
    # * a value of MaxWeight means that a chipper is required
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
            raise TypeError("Unexpected type for required_shell_id: " + str(
                value))
        self._required_shell_id = value

    @property
    def previous_shell_id(self):
        return self._previous_shell_id

    @previous_shell_id.setter
    def previous_shell_id(self, value):
        if value != None and not isinstance(value, int):
            raise TypeError("Unexpected type for previous_shell_id: " + str(
                value))
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

    @property
    def cost_func(self):
        return self._cost_func

    @cost_func.setter
    def cost_func(self, value):
        self._cost_func = value

# given a role requirements tree (with RoleRequirements or assignment tuples as leaves),
# yields all of the RoleRequiements objects
def iterate_role_requirements_tree_leaves(reqs_tree):
    if isinstance(reqs_tree, RoleRequirements) or isinstance(reqs_tree, tuple):
        yield reqs_tree
    else:
        for subtree in reqs_tree.values():
            yield from iterate_role_requirements_tree_leaves(subtree)


# This error is thrown by assign_roles() when given an impossible assignment scenario
class ImpossibleAssignmentError(RuntimeError):
    pass


# the munkres library doesn't like infinity, so we use this instead
MaxWeight = 10000000

# a default weight for preferring a chipper
# this is tunable
PreferChipper = 2.5


# uses the munkres/hungarian algorithm to find the optimal role assignments
# works by building a cost matrix for reach robot, role pair, then choosing the assignments to minimize total cost
# If no restraint-satisfying mass assignment exists, throws an ImpossibleAssignmentError
#
# role_reqs is a tree structure containing RoleRequirements
#
# returns a tree with the same structure as @role_reqs, but the leaf nodes have (RoleRequirements, OurRobot) tuples instead of just RoleRequirements objects
def assign_roles(robots, role_reqs):

    # check for empty request set
    if len(role_reqs) == 0:
        return {}

    # first we flatten the role_reqs tree structure into a simple list of RoleRequirements
    role_reqs_list = []
    tree_mapping = {
    }  # holds key paths so we can map the flattened list back into the tree at the end

    # The input and output to assign_roles() are in a tree form that maps to the behavior tree that we're assigning robots for
    # To do our calculations though, we need to have a flat list of RoleRequirements to match to our list of robots
    # The flatten_tree() method does this and keeps track of how to rebuild the tree so we can do so at the end
    def flatten_tree(tree, path_prefix=[]):
        valid_shell_ids = [bot.shell_id() for bot in robots]
        for key, subtree in tree.items():
            if isinstance(subtree, dict):
                flatten_tree(subtree, path_prefix + [key])
            elif isinstance(subtree, RoleRequirements):
                if subtree.required_shell_id is None or subtree.required_shell_id in valid_shell_ids:
                    role_reqs_list.append(subtree)
                tree_mapping[subtree] = path_prefix + [key]
            else:
                raise AssertionError("Unknown node type in role_reqs tree: " +
                                     str(subtree))

    flatten_tree(role_reqs)

    required_roles = [r for r in role_reqs_list if r.required]
    optional_roles = sorted(
        [r for r in role_reqs_list if not r.required],
        reverse=True,
        key=lambda r: r.priority)

    # logs the assignment parameters and raises an ImpossibleAssignmentError
    def fail(errStr):
        botsDesc = 'Robots:\n\t' + '\n\t'.join([str(bot) for bot in robots])
        rolesDesc = 'Roles:\n\t' + '\n\t'.join(
            [str(role) for role in role_reqs_list])
        logging.error('Failed Assignment:\n' + botsDesc + '\n' + rolesDesc)
        raise ImpossibleAssignmentError(
            "No assignments possible that satisfy all constraints")

    # make sure there's enough robots
    unassigned_role_requirements = [
    ]  # roles that won't be assigned because there aren't enough bots
    if len(required_roles) > len(robots):
        fail("More required roles than available robots")
    elif len(role_reqs_list) > len(robots):
        # remove the lowest priority optional roles so we have as many bots as roles we're trying to fill
        overflow = len(role_reqs_list) - len(robots)
        role_reqs_list = required_roles + optional_roles[0:-overflow]
        unassigned_role_requirements = optional_roles[len(optional_roles) -
                                                      overflow:]

    if len(robots) == 0:
        return {}

    # build the cost matrix
    cost_matrix = []
    for robot in robots:
        cost_row = []
        for req in role_reqs_list:
            cost = 0

            if req.required_shell_id != None and req.required_shell_id != robot.shell_id(
            ):
                cost = MaxWeight
            elif req.has_ball == True and robot.has_ball() == False:
                cost = MaxWeight
            elif req.require_kicking and (
                    robot.shell_id() == evaluation.double_touch.tracker()
                    .forbidden_ball_toucher() or not robot.kicker_works() or
                    not robot.ball_sense_works()):
                cost = MaxWeight
            else:
                if req.destination_shape != None:
                    cost += req.position_cost_multiplier * req.destination_shape.dist_to(
                        robot.pos)
                if req.previous_shell_id != None and req.previous_shell_id != robot.shell_id(
                ):
                    cost += req.robot_change_cost
                if not robot.has_chipper():
                    cost += req.chipper_preference_weight
                cost += req.cost_func(robot)

            # the munkres library freezes when given NaN values, causing our
            # whole program to hang and have to be restarted.  We check for it
            # here and raise an exception if there's a NaN.
            if (math.isnan(cost)):
                raise ArithmeticError(
                    "NaN value encountered when building role assignment cost matrix"
                )

            cost_row.append(cost)
        cost_matrix.append(cost_row)

    # solve
    solver = munkres.Munkres()
    indexes = solver.compute(cost_matrix)

    results = {}

    def insert_into_results(results, tree_mapping, role_reqs, robot):
        # get the keypath of this entry so we can insert back into the tree
        tree_path = tree_mapping[role_reqs]
        parent = results
        for key in tree_path[:-1]:
            if key not in parent:
                parent[key] = {}
            parent = parent[key]
        parent[tree_path[-1]] = (role_reqs, robot)

    # build assignments mapping
    assignments = {}
    total = 0
    for row, col in indexes:
        total += cost_matrix[row][col]

        bot = robots[row]
        reqs = role_reqs_list[col]

        # add entry to results tree
        insert_into_results(results, tree_mapping, reqs, bot)

    # insert None for each role that we didn't assign
    for reqs in unassigned_role_requirements:
        insert_into_results(results, tree_mapping, reqs, None)

    if total >= MaxWeight:
        fail("No assignments possible that satisfy all constraints")

    return results
