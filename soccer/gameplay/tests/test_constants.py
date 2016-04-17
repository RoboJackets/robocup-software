import unittest
import constants
import robocup


class TestConstants(unittest.TestCase):
    def test_our_goal_zone(self):
        # right in the center of the goal zone
        in_zone = robocup.Point(0, constants.Field.PenaltyDist / 2.0)

        out_zone = robocup.Point(0, constants.Field.Length / 2.0)

        self.assertTrue(constants.Field.OurGoalZoneShape.contains_point(
            in_zone))
        # self.assertFalse(constants.Field.OurGoalZoneShape.contains_point(out_zone))
