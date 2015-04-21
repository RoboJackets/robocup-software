import ui.play_config_tab
import logging
from PyQt5 import QtCore, QtWidgets
import main


def getMainWindow():
    win = None
    for widget in QtWidgets.QApplication.topLevelWidgets():
        if isinstance(widget, QtWidgets.QMainWindow):
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

    tabs = win.findChild(QtWidgets.QTabWidget, 'tabWidget')
    tabs.insertTab(0, pcTab, 'Plays')
    tabs.setCurrentIndex(0)

    logging.debug("Inserted PlayConfigTab at index zero")

    # bind the play label in the ui to the name of the current play
    play_name_label = win.findChild(QtWidgets.QLabel, 'current_play_name')
    main.root_play().play_changed.connect(play_name_label.setText)

    _has_setup_ui = True
