import unittest
import robocup
import sys


class TestConfig(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestConfig, self).__init__(*args, **kwargs)
        self.config = robocup.Configuration.FromRegisteredConfigurables()

    def test_config(self):
        name = "PathPlanner/selfAvoidRadius"
        avoid = self.config.nameLookup(name)
        self.assertTrue(isinstance(avoid.value, float))
        self.assertEqual(name, avoid.name)
