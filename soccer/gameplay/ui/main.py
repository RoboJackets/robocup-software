import logging
from PyQt5 import QtCore, QtWidgets
import main
import sys
import test_list


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
    global _testList

    if _has_setup_ui == True:
        logging.warn("ui setup() function called more than once")
        return

    win = getMainWindow()
    if win == None:
        raise AssertionError("Unable to get a reference to the main window")

    pcTab = win.findChild(QtWidgets.QTreeView, 'plays')
    testingTab = win.findChild(QtWidgets.QTreeView, 'allTestsTable')
    selectedTestsTable = win.findChild(QtWidgets.QListView, 'selectedTestsTable')
    _testList = test_list.TestList()

    # setup play config tab
    pcTab.setModel(main.play_registry())
    pcTab.expandAll()
    pcTab.resizeColumnToContents(0)

    logging.debug("Initialized PlayConfigTab")

    # setup testing tab
    testingTab.setModel(main.test_registry())
    testingTab.expandAll()
    testingTab.resizeColumnToContents(0)

    selectedTestsTable.setModel(_testList)


    logging.debug("Initialized TestConfigTab")

    # bind the play label in the ui to the name of the current play
    play_name_label = win.findChild(QtWidgets.QLabel, 'current_play_name')
    _defense_checkbox = win.findChild(QtWidgets.QCheckBox,
                                      'useDefenseCheckBox')

    main.root_play().play_changed.connect(play_name_label.setText)

    _has_setup_ui = True


def addTests():
    global _testList

    #TODO: implement custom listview for selectedTestsTable to support
    #status indicator, test results, and test data

    for item in main.test_registry():
        if (item.enabled):
            _testList.insert(item)

def runTests():
    print("UI RUN TESTS")
    for test in _testList.tList:

        # Enter Halt


        # Select Plays
        play_list = test.test_class.play_list
        playbook = []
        for play in play_list:
            playbook.append(play.split('/'))
        main.play_registry().load_playbook(playbook)

        # Place Entities
        #TODO: the rest of the owl

        # Enter Stop
        # Enter Normal Start (should this be changeable?)
        # Run till "next play button is pushed"

        # Store information
