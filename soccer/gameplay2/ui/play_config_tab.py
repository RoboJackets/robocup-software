from PyQt4 import QtCore, QtGui


class PlayConfigTab(QtGui.QTreeWidget):
    def __init__(self):
        super().__init__()
        self.setColumnCount(1);
