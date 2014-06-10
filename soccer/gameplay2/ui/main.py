import ui.play_config_tab
import logging
from PyQt4 import QtCore, QtGui


def getMainWindow():
    win = None
    for widget in QtGui.QApplication.topLevelWidgets():
        if isinstance(widget, QtGui.QMainWindow):
            win = widget
            break

    return win


# sets up the PlayConfigTab in the main soccer gui
_has_setup_ui = False
def setup():
    global _has_setup_ui

    if _has_setup_ui == True:
        logging.warn("ui setup() function called more than once")
        return

    win = getMainWindow()
    if win == None: raise AssertionError("Unable to get a reference to the main window")

    pcTab = ui.play_config_tab.PlayConfigTab()
    pcTab.setObjectName('play_config')

    tabs = win.findChild(QtGui.QTabWidget, 'tabWidget')
    tabs.insertTab(0, pcTab, 'Plays')

    logging.debug("Inserted PlayConfigTab at index zero")

    _has_setup_ui = True
