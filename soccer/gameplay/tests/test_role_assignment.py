import unittest
import role_assignment
import robocup


class TestRoleAssignment(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestRoleAssignment, self).__init__(*args, **kwargs)
        # Some objects initialized below depend on the config system being setup
        self.config = robocup.Configuration.FromRegisteredConfigurables()
        self.system_state = robocup.SystemState()

    def test_pos_cost(self):
        """Ensure that when requirements specify a target position, it is taken
        into account in assignment"""

        bot1 = robocup.OurRobot(1, self.system_state)
        bot1.set_pos_for_testing(robocup.Point(1, 6))

        bot2 = robocup.OurRobot(2, self.system_state)
        bot2.set_pos_for_testing(robocup.Point(2, 3))

        req1 = role_assignment.RoleRequirements()
        req1.destination_shape = robocup.Point(1, 7)

        req2 = role_assignment.RoleRequirements()
        req2.destination_shape = robocup.Point(3, 4)

        req_tree = {'role1': req1, 'role2': req2}
        assignments = role_assignment.assign_roles([bot1, bot2], req_tree)
        self.assertEqual(len(assignments), 2)
        self.assertEqual(assignments['role1'][1], bot1)
        self.assertEqual(assignments['role2'][1], bot2)


    def test_not_enough_bots(self):
        """If there's not enough robots to do an assignment, it should raise an error"""

        bot1 = robocup.OurRobot(1, self.system_state)
        bot1.set_pos_for_testing(robocup.Point(1, 6))

        req1 = role_assignment.RoleRequirements()
        req1.pos = robocup.Point(1, 7)
        req1.required = True

        req2 = role_assignment.RoleRequirements()
        req2.pos = robocup.Point(3, 4)
        req2.required = True

        req_tree = {'role1': req1, 'role2': req2}
        self.assertRaises(role_assignment.ImpossibleAssignmentError,
            role_assignment.assign_roles,
            [bot1],
            req_tree)
