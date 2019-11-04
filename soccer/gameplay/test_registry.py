from PyQt5 import QtCore
import logging


## Holds references to all Test subclasses and their enabled state
# The play registry keeps a tree of all tests in the 'tests' folder (and its subfolders)
# Our old system required programmatically registering tests into categories, but
# the new system just uses the filesystem hierarchy for this.
#
# The registry has methods for loading and unloading tests (for when files change on disk)
#
# It also tracks which tests are enabled
#
# This is a subclass of QAbstractItemModel so that we can easily attach a UI
class TestRegistry(QtCore.QAbstractItemModel):
    def __init__(self):
        super().__init__()
        self._root = TestRegistry.Category(None, "")

    @property
    def root(self):
        return self._root

    # the module path is a list
    # for a demo test called RunAround, module_path = ['demo', 'run_around']
    # (note that we left out 'gameplay-tests' - every test is assumed to be in a descendent module of it)
    def insert(self, module_path, test_class):
        category = self.root

        # iterate up to the last one (the last one is just an underscored,
        # lowercased version of the test's name and we don't display it in the tree)
        for module in module_path[:-1]:
            if not category.has_child_with_name(module):
                subcategory = TestRegistry.Category(category, module)
                category.append_child(subcategory)
            category = category[module]

        testNode = TestRegistry.Node(module_path[-1], test_class)
        category.append_child(testNode)

    def load_testbook(self, list_of_tests):
        self.clear()
        for test in list_of_tests:
            node = self.node_for_module_path(test)
            if node is not None:
                node.enabled = True
            else:
                logging.warn("Attempt to load non-existent test " + '/'.join(
                    test) + " from testbook.")

    def delete(self, module_path):
        node = self.node_for_module_path(module_path)
        del node.parent[node.name]

        # remove any categories where this test was the only entry
        node = node.parent
        while node.parent is not None:
            if len(node.children) == 0:
                del node.parent[node.name]
                node = node.parent
            else:
                break


    # iterates over all of the Nodes registered in the tree
    def __iter__(self):
        def _recursive_iter(category):
            for child in category.children:
                if isinstance(child, TestRegistry.Node):
                    yield child
                else:
                    yield from _recursive_iter(child)

        return _recursive_iter(self.root)

    def __contains__(self, test_class):
        for node in self:
            if node.test_class == test_class:
                return True
        return False

    def __str__(self):
        def _cat_str(category, indent):
            desc = ""
            for child in category.children:
                if isinstance(child, TestRegistry.Node):
                    desc += "    " * indent
                    desc += str(child)
                else:
                    desc += "    " * indent + child.name + ':' + '\n'
                    desc += _cat_str(child, indent + 1)
                desc += '\n'
            return desc[:-1]  # delete trailing newline

        return "TestRegistry:\n-------------\n" + _cat_str(self.root, 0)

    # module_path is a list like ['demo', 'my_demo']
    # returns a Node or None if it can't find it
    def node_for_module_path(self, module_path):
        category = self.root
        for module_name in module_path[:-1]:
            category = category[module_name]
            if category is None:
                return None

        for child in category.children:
            if isinstance(child, TestRegistry.Node):
                if child.module_name == module_path[-1]:
                    return child

        return None

    ## Categories correspond to filesystem directories
    class Category():
        def __init__(self, parent, name):
            super().__init__()

            self._name = name
            self._children = list()
            self.parent = parent

        @property
        def name(self):
            return self._name

        @property
        def module_name(self):
            return self.name

        def __delitem__(self, name):
            for idx, child in enumerate(self.children):
                if child.name == name:
                    del self.children[idx]
                    return
            raise KeyError("Testing System attempted to delete a child node that doesn't exist")

        def append_child(self, child):
            self.children.append(child)
            child.parent = self

        def __getitem__(self, name):
            for child in self.children:
                if child.name == name:
                    return child
            return None

        def has_child_with_name(self, name):
            return self[name] != None

        # @children is a list
        @property
        def children(self):
            return self._children

        @property
        def row(self):
            if self.parent is not None:
                return self.parent.children.index(self)
            else:
                return 0

    class Node():
        def __init__(self, module_name, test_class):
            self._module_name = module_name
            self.enabled = False
            self.test_class = test_class
            self.parent = None

            self.status = None
            self.information = None

        @property
        def name(self):
            return self.test_class.__name__

        @property
        def module_name(self):
            return self._module_name

        def __str__(self):
            return self.test_class.__name__

    # Note: a lot of the QAbstractModel-specific implementation is borrowed from here:
    # http://www.hardcoded.net/articles/using_qtreeview_with_qabstractitemmodel.htm

    # one column: 'Test'
    def columnCount(self, parent):
        return 1

    def flags(self, index):
        if index.column() == 0:
            return QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsUserCheckable

    def data(self, index, role):
        if not index.isValid():
            return None
        node = index.internalPointer()
        if role == QtCore.Qt.DisplayRole:
            if index.column() == 0:
                return node.name
            elif index.column() == 1:
                if isinstance(node, TestRegistry.Node):
                    return str(node.test_class.name())
                else:
                    return None
        elif role == QtCore.Qt.CheckStateRole and isinstance(
                node, TestRegistry.Node):
            if index.column() == 0:
                return node.enabled
            else:
                return None

        return None

    def rowCount(self, parent):
        if not parent.isValid():
            return len(self.root.children)
        node = parent.internalPointer()
        if isinstance(node, TestRegistry.Node):
            return 0
        else:
            return len(node.children)

    def parent(self, index):
        if not index.isValid():
            return QtCore.QModelIndex()
        else:
            node = index.internalPointer()
            if node == None:
                return QtCore.QModelIndex()
            elif node.parent == None:
                parentRow = 0
            else:
                parentRow = node.parent.row
            return self.createIndex(parentRow, index.column(), node.parent)

    def index(self, row, column, parent):
        if not parent.isValid():
            return self.createIndex(row, column, self.root.children[row])
        parentNode = parent.internalPointer()
        if parentNode != None:
            return self.createIndex(row, column, parentNode.children[row])
        else:
            return QtCore.QModelIndex()

    def headerData(self, section, orientation, role):
        if orientation == QtCore.Qt.Horizontal and role == QtCore.Qt.DisplayRole:
            if section == 0:
                return 'Test'
        return None


    def setData(self, index, value, role):
        if role == QtCore.Qt.CheckStateRole:
            if index.isValid():
                testNode = index.internalPointer()
                if not isinstance(testNode, TestRegistry.Node):
                    raise AssertionError(
                        "Only Test Nodes should be checkable...")
                testNode.enabled = not testNode.enabled
                self.dataChanged.emit(index, index)
                return True
        return False

    def updateModel(self):
        self.dataChanged.emit(QtCore.QModelIndex(), QtCore.QModelIndex())
