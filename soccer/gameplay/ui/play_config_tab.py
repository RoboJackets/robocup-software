from PyQt4 import QtCore, QtGui
import main


class PlayConfigTab(QtGui.QTreeView):
    def __init__(self):
        super().__init__()
        self.setModel(main.play_registry())
        self.expandAll()
        self.resizeColumnToContents(0)
