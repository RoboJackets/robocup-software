""" This file contains custom checkers for pylint.
"""

from typing import List

import astroid.node_classes
import astroid.nodes
from pylint.checkers import BaseChecker
from pylint.interfaces import IAstroidChecker
from pylint.lint.pylinter import PyLinter


class NoFromImportChecker(BaseChecker):
    """This checker checks that from * import * isn't used for any modules in the stp
    package.
    """

    __implements__ = IAstroidChecker

    DISPLAYED_MSG = (
        "Don't use 'from ... import ...' for any modules from the stp package."
    )
    MSG_ID = "stp-from-import"
    MESSAGE_HELP = (
        "'from ... import ...' shouldn't be used for importing any "
        "modules / symbols from the stp package as it breaks hot reloading."
    )

    name = "stp-from-import"
    priority = -100
    msgs = {"C8001": (DISPLAYED_MSG, MSG_ID, MESSAGE_HELP)}

    def __init__(self, linter: PyLinter):
        super().__init__(linter)

        self.is_testing_module: List[bool] = []

    def visit_module(self, node: astroid.nodes.Module) -> None:
        """Visit method for astroid.nodes.Module."""
        is_testing_module = "test" in node.name
        self.is_testing_module.append(is_testing_module)

    def leave_module(self, node: astroid.nodes.Module) -> None:
        """Leave method for astroid.nodes.Module."""
        self.is_testing_module.pop()

    def visit_importfrom(self, node: astroid.nodes.ImportFrom) -> None:
        """Visit method for astroid.nodes.ImportFrom."""
        if node.modname and node.modname == "stp":
            if not self.__is_testing_module():
                self.add_message(
                    NoFromImportChecker.MSG_ID,
                    node=node,
                )

    def __is_testing_module(self) -> bool:
        """Returns whether the current module is for tests.
        :return: True if the current module is for tests,
        """
        return any(self.is_testing_module)


def register(linter: PyLinter):
    """Registers the linter for pylint."""
    linter.register_checker(NoFromImportChecker(linter))
