from PyQt5 import QtCore

class TestList(QtCore.QAbstractListModel):
    tList = []

    def __init__(self):
        super().__init__()

    def insert(self, item):
        self.tList.append(item)
        indexStart = self.createIndex(0, 0)
        indexEnd = self.createIndex(len(self.tList) - 1, 0)
        self.dataChanged.emit(indexStart, indexEnd)

    def rowCount(self, parent):
        return len(self.tList)

    def data(self, index, role):
        if not index.isValid():
            return None
        node = index.internalPointer()
        if role == QtCore.Qt.DisplayRole:
            if index.column() == 0:
                print("name: ", node.name)
                return node.name
        return None

    def index(self, row, column, parent = None):
        print("indexing to ", row, ", ", column)
        if row < len(self.tList) and column == 0:
            return self.createIndex(row, column, self.tList[row])
        return QtCore.QModelIndex()
    '''
    def flags(self, index):
        if index.column() == 0:
            return QtCore.Qt.ItemIsSelectable
        return QtCore.Qt.NoItemFlags
    '''
