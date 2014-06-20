from play import *
from behavior import *
import plays.stopped
import logging
from PyQt4 import QtCore
import main
import tactics.roles.goalie
import traceback


# the RootPlay is basically the python-side of the c++ GameplayModule
# it coordinates the selection of the 'actual' play and handles the goalie behavior
class RootPlay(Play, QtCore.QObject):

    def __init__(self):
        QtCore.QObject.__init__(self)
        Play.__init__(self, continuous=True)
        self._play = None
        self._goalie_id = None

        self.add_transition(Behavior.State.start, Behavior.State.running, lambda: True, 'immediately')


    play_changed = QtCore.pyqtSignal("QString")


    def execute_running(self):
        # TODO: do goalie stuff

        enabled_plays = main.play_registry().get_enabled_plays()

        if main.game_state.is_stopped() and self.play.__class__ != plays.stopped.Stopped:
            self.play = plays.stopped.Stopped()
            logging.info("Switched to Stopped play")
        elif not main.game_state.is_stopped() and self.play.__class__ == plays.stopped.Stopped:
            self.play = None
        elif self.play != None and self.play.__class__ not in enabled_plays:
            self.play = None

        if self.play == None:
            # select the play with the largest value for score()
            try:
                if len(enabled_plays) > 0:
                    play_class = max(enabled_plays, key=lambda p: p.score())
                    self.play = play_class()
                else:
                    self.play = None
            except Exception as e:
                logging.error("Exception occurred during play selection: " + str(e))
                traceback.print_exc()
            if self.play != None:
                logging.info("Chose new play: '" + self.play.__class__.__name__ + "'")

        if self.play != None:
            try:
                self.play.robots = self.robots
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


    # the c++ GameplayModule reaches through the language portal and sets this
    # note that in c++, a value of -1 indicates no assigned goalie, in python we represent the same thing with None
    @property
    def goalie_id(self):
        return self._goalie_id
    @goalie_id.setter
    def goalie_id(self, value):
        self._goalie_id = None if value == -1 else value
        logging.info("goalie_id set to: " + str(self._goalie_id))


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
