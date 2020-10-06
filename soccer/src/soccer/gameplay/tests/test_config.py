import unittest
import robocup
import sys


class TestConfig(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestConfig, self).__init__(*args, **kwargs)
        self.config = robocup.Configuration.FromRegisteredConfigurables()

    def test_config(self):
        name = "PathPlanner/RRT/EnableDebugDrawing"
        avoid = self.config.nameLookup(name)
        self.assertTrue(isinstance(avoid.value, bool))
        self.assertEqual(name, avoid.name)
