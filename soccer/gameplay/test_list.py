from PyQt5 import QtCore
import test_registry
from enum import Enum

class Status(Enum):
    idle = 0
    running = 1
    completed = 2

class TestNode(test_registry.TestRegistry.Node):
    def __init__(self, registryNode):
        super().__init__(registryNode._module_name, registryNode.test_class)
        self.reset()

    def reset(self):
        self.test = self.test_class()
        self.status = Status.idle
        self.results = []

class TestList(QtCore.QAbstractListModel):
    def __init__(self, selectedTestsTable):
        super().__init__()
        self.tList = []
        self.selectedTestsTable = selectedTestsTable

    def insert(self, item):
        self.tList.append(TestNode(item))
        index = self.createIndex(len(self.tList) - 1, 0)
        self.dataChanged.emit(index, index)

    def remove(self, node):
        i = self.tList.index(node)
        del self.tList[i]

        index = self.createIndex(i, i)

        # After a lot of experimentation this is the closest that I
        # can get to the correct implementation of this.
        self.dataChanged.emit(index, index)
        self.beginRemoveRows(self.createIndex(0,0), i, i)
        self.rowsRemoved.emit(self.createIndex(0,0), i, i)
        self.endRemoveRows()

        return i

    def rowCount(self, parent):
        return len(self.tList)

    def data(self, index, role = None):
        if not index.isValid():
            return None
        node = index.internalPointer()

        # This if is annoyingly necessary
        # QT somehow calls this on garbage data
        # during the first iteration after a remove
        if isinstance(node, TestNode):
            if role == QtCore.Qt.DisplayRole:
                if index.column() == 0:
                    return node.name
            if role == None:
                if index.column() == 0:
                    return node

        return None

    def index(self, row, column = 0, parent = None):
        if row < len(self.tList) and column == 0:
            return self.createIndex(row, column, self.tList[row])
        return QtCore.QModelIndex()


    def size(self):
        return len(self.tList)

    def getSelectedNode(self):
        selectionModel = self.selectedTestsTable.selectionModel()
        return self.data(selectionModel.selectedIndexes()[0])

    def selectIndex(self, index):
        # Set the specific index to be selected

        selectionModel = self.selectedTestsTable.selectionModel()
        selectionModel.setCurrentIndex(self.index(index), QtCore.QItemSelectionModel.ClearAndSelect)

    def removeSelectedNode(self):
        selectionModel = self.selectedTestsTable.selectionModel()
        return self.remove(self.data(selectionModel.selectedIndexes()[0]))
