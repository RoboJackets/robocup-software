import unittest
import play_registry
import plays.testing.line_up


class TestPlayRegistry(unittest.TestCase):
    def test_insert(self):
        pr = play_registry.PlayRegistry()
        pr.insert(['line_up'], plays.testing.line_up.LineUp)
        self.assertTrue(plays.testing.line_up.LineUp in pr)

    def test_delete(self):
        pr = play_registry.PlayRegistry()
        pr.insert(['line_up'], plays.testing.line_up.LineUp)
        pr.delete(['line_up'])
        self.assertFalse(plays.testing.line_up.LineUp in pr)

    def test_nested_insert(self):
        pr = play_registry.PlayRegistry()
        pr.insert(['demo', 'line_up'], plays.testing.line_up.LineUp)
        self.assertTrue(plays.testing.line_up.LineUp in pr)

    def test_delete_last_item_in_category(self):
        """When the last play in a category is deleted, the category should be removed"""

        pr = play_registry.PlayRegistry()
        pr.insert(['demo', 'line_up'], plays.testing.line_up.LineUp)
        self.assertEqual(len(pr.root.children), 1)
        pr.delete(['demo', 'line_up'])
        self.assertEqual(len(pr.root.children), 0)
