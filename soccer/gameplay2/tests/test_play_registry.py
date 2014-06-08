import unittest
import play_registry
import plays.line_up


class TestPlayRegistry(unittest.TestCase):
    def setUp(self):
        self.play_registry = play_registry.PlayRegistry()
        self.play_registry.insert(['line_up'], plays.line_up.LineUp)

    def test_insert(self):
        self.assertTrue(plays.line_up.LineUp in self.play_registry)
