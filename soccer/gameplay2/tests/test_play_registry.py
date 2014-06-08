import unittest
import play_registry
import plays.line_up


class TestPlayRegistry(unittest.TestCase):
    def test_insert(self):
        pr = play_registry.PlayRegistry()
        pr.insert(['line_up'], plays.line_up.LineUp)
        self.assertTrue(plays.line_up.LineUp in pr)

    def test_delete(self):
        pr = play_registry.PlayRegistry()
        pr.insert(['line_up'], plays.line_up.LineUp)
        pr.delete(['line_up'], plays.line_up.LineUp)
        self.assertFalse(plays.line_up.LineUp in pr)

    def test_nested_insert(self):
        pr = play_registry.PlayRegistry()
        pr.insert(['demo', 'line_up'], plays.line_up.LineUp)
        self.assertTrue(plays.line_up.LineUp in pr)

    def test_delete_last_item_in_category(self):
        """When the last play in a category is deleted, the category should be removed"""

        pr = play_registry.PlayRegistry()
        pr.insert(['demo', 'line_up'], plays.line_up.LineUp)
        self.assertTrue('demo' in pr.root)
        pr.delete(['demo', 'line_up'], plays.line_up.LineUp)
        self.assertFalse('demo' in pr.root)
