from PyQt5 import QtCore
import logging


## Holds references to all Play subclasses and their enabled state
# The play registry keeps a tree of all plays in the 'plays' folder (and its subfolders)
# Our old system required programmatically registering plays into categories, but
# the new system just uses the filesystem hierarchy for this.
#
# The registry has methods for loading and unloading plays (for when files change on disk)
#
# It also tracks which plays are enabled
#
# This is a subclass of QAbstractItemModel so that we can easily attach a UI
class PlayRegistry(QtCore.QAbstractItemModel):
    def __init__(self):
        super().__init__()
        self._root = PlayRegistry.Category(None, "")

    @property
    def root(self):
        return self._root

    # the module path is a list
    # for a demo play called RunAround, module_path = ['demo', 'run_around']
    # (note that we left out 'plays' - every play is assumed to be in a descendent module of it)
    def insert(self, module_path, play_class):
        category = self.root

        # iterate up to the last one (the last one is just an underscored,
        # lowercased version of the play's name and we don't display it in the tree)
        for module in module_path[:-1]:
            if not category.has_child_with_name(module):
                subcategory = PlayRegistry.Category(category, module)
                category.append_child(subcategory)
            category = category[module]

        playNode = PlayRegistry.Node(module_path[-1], play_class)
        # if playNode.module_name in category:
        #     raise AssertionError("There's already a play registered for the given module path")
        category.append_child(playNode)

        # note: this is a shitty way to do this - we should really only reload part of the model
        self.modelReset.emit()

    def load_playbook(self, list_of_plays):
        self.clear()
        for play in list_of_plays:
            node = self.node_for_module_path(play)
            if node is not None:
                node.enabled = True
            else:
                logging.warn("Attempt to load non-existent play " + '/'.join(
                    play) + " from playbook.")

        # note: this is a shitty way to do this - we should really only reload part of the model
        self.modelReset.emit()

    def delete(self, module_path):
        node = self.node_for_module_path(module_path)
        del node.parent[node.name]

        # remove any categories where this play was the only entry
        node = node.parent
        while node.parent is not None:
            if len(node.children) == 0:
                del node.parent[node.name]
                node = node.parent
            else:
                break

        # note: this is a shitty way to do this - we should really only reload part of the model
        self.modelReset.emit()

    def clear(self):
        enabled_plays = self.get_enabled_plays_paths()
        for play in enabled_plays:
            node = self.node_for_module_path(play)
            if node is not None:
                node.enabled = False
            else:
                logging.warn("Attempt to clear non-existent play " + '/'.join(
                    play) + " from play registry.")

    # cache and calculate the score() function for each play class
    def recalculate_scores(self):
        self.root.recalculate_scores(self)

    ## Get a list of all plays in the tree that are currently enabled
    def get_enabled_plays_and_scores(self):
        return [(node.play_class, node.last_score)
                for node in self if node.enabled]

    ## Returns a list of module paths for the currently-enabled plays
    # The module path is a list or tuple giving the path the the play's python module
    # For example: ['testing', 'test_pivot_kick']
    def get_enabled_plays_paths(self):
        enabled_plays = []

        for node in self:
            if node.enabled:
                play_path = []

                curr_node = node
                while curr_node is not None:
                    if curr_node.module_name:
                        play_path.insert(0, curr_node.module_name)
                    curr_node = curr_node.parent

                enabled_plays.append(play_path)

        return enabled_plays

    # iterates over all of the Nodes registered in the tree
    def __iter__(self):
        def _recursive_iter(category):
            for child in category.children:
                if isinstance(child, PlayRegistry.Node):
                    yield child
                else:
                    yield from _recursive_iter(child)

        return _recursive_iter(self.root)

    def __contains__(self, play_class):
        for node in self:
            if node.play_class == play_class:
                return True
        return False

    def __str__(self):
        def _cat_str(category, indent):
            desc = ""
            for child in category.children:
                if isinstance(child, PlayRegistry.Node):
                    desc += "    " * indent
                    desc += str(child)
                else:
                    desc += "    " * indent + child.name + ':' + '\n'
                    desc += _cat_str(child, indent + 1)
                desc += '\n'
            return desc[:-1]  # delete trailing newline

        return "PlayRegistry:\n-------------\n" + _cat_str(self.root, 0)

    # module_path is a list like ['demo', 'my_demo']
    # returns a Node or None if it can't find it
    def node_for_module_path(self, module_path):
        category = self.root
        for module_name in module_path[:-1]:
            category = category[module_name]
            if category is None:
                return None

        for child in category.children:
            if isinstance(child, PlayRegistry.Node):
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

        # Instructs all child nodes to recalculate their scores.
        # if a child node returns True indicating that the score value changed, we
        # emit the "dataChanged" signal with the corresponding node index
        def recalculate_scores(self, model):
            for child in self._children:
                if child.recalculate_scores(model):
                    row = child.parent.children.index(child)
                    col = 1
                    parent = child.parent
                    index = model.createIndex(row, col, child)
                    model.dataChanged.emit(index, index)
            return False

        def __delitem__(self, name):
            for idx, child in enumerate(self.children):
                if child.name == name:
                    del self.children[idx]
                    return
            raise KeyError("Attempt to delete a child node that doesn't exist")

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
        def __init__(self, module_name, play_class):
            self._module_name = module_name
            self._last_score = float("inf")
            self.enabled = False
            self.play_class = play_class
            self.parent = None

        @property
        def name(self):
            return self.play_class.__name__

        @property
        def module_name(self):
            return self._module_name

        # recalculates and caches the score value for the play
        # returns True if the value changed and False otherwise
        def recalculate_scores(self, model):
            prev = self._last_score
            self._last_score = self.play_class.score()
            return prev != self._last_score

        @property
        def last_score(self):
            return self._last_score

        def __str__(self):
            return self.play_class.__name__ + " " + (
                "[ENABLED]" if self.enabled else "[DISABLED]")

    # Note: a lot of the QAbstractModel-specific implementation is borrowed from here:
    # http://www.hardcoded.net/articles/using_qtreeview_with_qabstractitemmodel.htm

    # two columsn: 'Play', 'Score'
    def columnCount(self, parent):
        return 2

    def flags(self, index):
        if index.column() == 0:
            return QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsUserCheckable | QtCore.Qt.ItemIsEditable
        else:
            return QtCore.Qt.ItemIsEnabled

    def data(self, index, role):
        if not index.isValid():
            return None
        node = index.internalPointer()
        if role == QtCore.Qt.DisplayRole:
            if index.column() == 0:
                return node.name
            elif index.column() == 1:
                if isinstance(node, PlayRegistry.Node):
                    return str(node.play_class.score())
                else:
                    return None
        elif role == QtCore.Qt.CheckStateRole and isinstance(
                node, PlayRegistry.Node):
            if index.column() == 0:
                return node.enabled
            else:
                return None
        return None

    def rowCount(self, parent):
        if not parent.isValid():
            return len(self.root.children)
        node = parent.internalPointer()
        if isinstance(node, PlayRegistry.Node):
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
                return 'Play'
            else:
                return 'Score'
        return None

    # this is implemented so we can enable/disable plays from the gui
    def setData(self, index, value, role):
        if role == QtCore.Qt.CheckStateRole:
            if index.isValid():
                playNode = index.internalPointer()
                if not isinstance(playNode, PlayRegistry.Node):
                    raise AssertionError(
                        "Only Play Nodes should be checkable...")
                playNode.enabled = not playNode.enabled
                self.dataChanged.emit(index, index)
                return True
        return False
