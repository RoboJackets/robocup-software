from PyQt5 import QtCore, QtWidgets
import logging
import test_list
from playbook import load_from_file
import constants
import robocup


class TestSystem:
    def __init__(self, play_registry, test_registry, win):
        self._play_registry = play_registry
        self._test_registry = test_registry

        testingTab = win.findChild(QtWidgets.QTreeView, 'allTestsTable')
        selectedTestsTable = win.findChild(QtWidgets.QListView,
                                           'selectedTestsTable')

        self._testList = test_list.TestList(selectedTestsTable)
        self._testIndex = 0
        self._testNode = None

        testingTab.setModel(self._test_registry)
        testingTab.expandAll()
        testingTab.resizeColumnToContents(0)

        selectedTestsTable.setModel(self._testList)

        logging.debug("Initialized TestConfigTab")

    def addTests(self):
        #TODO: implement custom listview for selectedTestsTable to support
        #status indicator, test results, and test data

        for item in self._test_registry:
            if (item.enabled):
                self._testList.insert(item)
                item.enabled = False

        self._test_registry.updateModel()

    def removeTest(self):
        index = self._testList.removeSelectedNode()

        if index < self._testIndex:
            # If the removed test effects the current index, move it to the correct position
            self._testIndex -= 1
        elif index == self._testIndex and self._testNode.status == test_list.Status.running:
            # If the running test was removed, load the next one
            self.loadTest()

    def loadTest(self):
        if self._testList.size() == 0:
            # No tests selected, not running
            return False
        elif self._testIndex >= self._testList.size():
            # test index out of range, start from the beginning
            self._testIndex = 0

        # This initializes a fresh version of the test
        # Even if it has already been run before
        self._testNode = self._testList.tList[self._testIndex]
        self._testNode.reset()

        # Set required plays to selected in the play registry
        plays = self.parsePlayList(self._testNode.test.play_list)
        print("final: ", plays)

        # If there are any plays in the play list, set them as the current playbook
        if(len(plays) > 0):
            self._play_registry.load_playbook(plays)

        self._testNode.status = test_list.Status.running
        self._testList.selectIndex(self._testIndex)

        return True

    def getNextCommand(self):
        if self._testNode is None or not self._testNode.test.start_commands:
            # Test Node does not exist or there are no start_commands
            return None

        return self._testNode.test.start_commands.pop(0)

    def nextTest(self):
        # If the test is running, go to next test, otherwise leave index the same
        if self._testNode is not None and self._testNode.status != test_list.Status.idle:
            self._testNode.status = test_list.Status.completed
            self._testList.tList[self._testIndex] = self._testNode

            # Increment testIndex, if it is too large wrap around
            self._testIndex += 1
            if self._testIndex >= self._testList.size():
                self._testIndex = 0
        # loadTest() is called from the C++ side if it is necessary

    def getTestOurRobots(self):
        rtrn = []
        for robot in self._testNode.test.ourRobots:
            rtrn.append([robot.pos.x, robot.pos.y, robot.angle])

        return rtrn

    def getTestTheirRobots(self):
        rtrn = []
        for robot in self._testNode.test.theirRobots:
            rtrn.append([robot.pos.x, robot.pos.y, robot.angle])

        return rtrn

    def getTestBall(self):
        pos = self._testNode.test.ballPosition
        vel = self._testNode.test.ballVelocity

        return [pos.x, pos.y, vel.x, vel.y]

    def parsePlayList(self, play_list):
        plays = []

        for play in play_list:
            play = play.strip()

            if play:
                #Check to see if the entry is a playbook file
                if(".pbk" in play):
                    plays.extend(load_from_file("./soccer/gameplay/playbooks/" + play))
                else:
                    plays.append(play.split('/'))

        #Currently duplicates are allowed and it dosen't seem to cause any problems

        return plays
