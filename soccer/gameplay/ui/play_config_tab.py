from PyQt5 import QtCore, QtWidgets
import main


class PlayConfigTab(QtWidgets.QTreeView):
    """The play config tab allows users to enable and disable plays"""
    
    def __init__(self):
        super().__init__()
        self.setModel(main.play_registry())
        self.expandAll()
        self.resizeColumnToContents(0)
