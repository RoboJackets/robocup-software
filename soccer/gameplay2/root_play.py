from play import *
from behavior import *
import logging
from PyQt4 import QtCore
import main
import tactics.roles.goalie


# the RootPlay is basically the python-side of the c++ GameplayModule
# it coordinates the selection of the 'actual' play and handles the goalie behavior
class RootPlay(Play, QtCore.QObject):

    def __init__(self):
        QtCore.QObject.__init__(self)
        Play.__init__(self, continuous=True)
        self._play = None

        self.add_transition(Behavior.State.start, Behavior.State.running, lambda: True, 'immediately')


    play_changed = QtCore.pyqtSignal("QString")


    def execute_running(self):
        # TODO: do goalie stuff
        if self.play == None:
            # select the play with the largest value for score()
            try:
                play_class = max(main.play_registry().get_enabled_plays(), key=lambda p: p.score())
                self.play = play_class()
            except Exception as e:
                logging.error("Exception occurred during play selection: " + str(e))
            
            logging.info("Chose new play: '" + self.play.__class__.__name__ + "'")

        if self.play != None:
            try:
                self.play.run()
            except Exception as e:
                logging.error("Play '" + self.play.__class__.__name__ + "' encountered exception: " + str(e) + ". aborting and reselecting play...")


    def on_exit_running(self):
        self._play = None


    # this is used to force a reselection of a play
    def drop_current_play(self):
        self.play = None


    # this is called when the goalie behavior must be reloaded (for example when the goalie.py file is modified)
    def drop_goalie_behavior(self):
        self.goalie_behavior = None


    @property
    def play(self):
        return self._play
    @play.setter
    def play(self, value):
        self._play = value
        if self._play != None:
            self._play.robots = self.robots
        # change notification so ui can update if necessary
        self.play_changed.emit(self._play.__class__.__name__ if self._play != None else "(No Play)")


    @property
    def goalie_behavior(self):
        return self._goalie_behavior
    @goalie_behavior.setter
    def goalie_behavior(self, value):
        self._goalie_behavior = value


    @Play.robots.setter
    def robots(self, robots):
        # FIXME: assign goalie

        #FIXME: call superclass setter?
        self._robots = robots

        # pass robots to play
        if self.play != None:
            self.play.robots = robots
