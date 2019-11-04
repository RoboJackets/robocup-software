import logging
from PyQt5 import QtCore, QtWidgets
import main
import robocup
import sys
import test_system


def getMainWindow():
    win = None
    for widget in QtWidgets.QApplication.topLevelWidgets():
        if isinstance(widget, QtWidgets.QMainWindow):
            win = widget
            break

    return win

# sets up the PlayConfigTab in the main soccer gui
_has_setup_ui = False

_defense_checkbox = None
def defenseEnabled():
    if _defense_checkbox is None:
        return False
    return _defense_checkbox.isChecked()


def setup():
    global _has_setup_ui
    global _defense_checkbox
    global _tests

    if _has_setup_ui == True:
        logging.warn("ui setup() function called more than once")
        return

    win = getMainWindow()
    if win == None:
        raise AssertionError("Unable to get a reference to the main window")

    pcTab = win.findChild(QtWidgets.QTreeView, 'plays')

    # setup play config tab
    pcTab.setModel(main.play_registry())
    pcTab.expandAll()
    pcTab.resizeColumnToContents(0)

    logging.debug("Initialized PlayConfigTab")


    logging.debug("Initialized TestConfigTab")

    # bind the play label in the ui to the name of the current play
    play_name_label = win.findChild(QtWidgets.QLabel, 'current_play_name')
    _defense_checkbox = win.findChild(QtWidgets.QCheckBox,
                                      'useDefenseCheckBox')

    main.root_play().play_changed.connect(play_name_label.setText)

    _tests = test_system.TestSystem(main.play_registry(), main.test_registry(), win)

    _has_setup_ui = True
