from play import *
from behavior import *
import plays.stopped
import logging
from PyQt4 import QtCore
import main
import tactics.positions.goalie
import role_assignment
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

        # if a play fails for some reason, we can temporarily blacklist it, which removes it from play
        # selection for the next iteration, then enables it again
        self.temporarily_blacklisted_play_class = None


    play_changed = QtCore.pyqtSignal("QString")

    def execute_running(self):

        # Play Selection
        ################################################################################

        if main.game_state().is_stopped():
            if not isinstance(self.play, plays.stopped.Stopped):
                logging.info("Running 'Stopped' play due to game state change")
                self.play = plays.stopped.Stopped()
        elif main.game_state().is_halted():
            self.play = None
        else:
            enabled_plays = main.play_registry().get_enabled_plays()

            # handle temporary blacklisting
            # we remove the blacklisted play class from selection for this iteration, then unblacklist it
            enabled_plays = [p for p in enabled_plays if p != self.temporarily_blacklisted_play_class]
            self.temporarily_blacklisted_play_class = None

            # see if we need to kill current play
            if self.play != None:
                if self.play.__class__ not in enabled_plays:
                    logging.info("Current play '" + self.play.__class__.__name__ + "'" + " no longer enabled, aborting")
                    self.play = None
                elif self.play.is_done_running():
                    logging.info("Current play '" + self.play.__class__.__name__ + "'" + " finished running")
                    self.play = None

            if self.play == None:
                try:
                    if main.game_state().is_halted():
                        # don't run a play, we're halted
                        pass
                    if len(enabled_plays) > 0:
                        # select the play with the largest value for score()
                        play_class = min(enabled_plays, key=lambda p: p.score())
                        self.play = play_class()
                    else:
                        # there's no available plays to run
                        pass
                except Exception as e:
                    logging.error("Exception occurred during play selection: " + str(e))
                    traceback.print_exc()
                if self.play != None:
                    logging.info("Chose new play: '" + self.play.__class__.__name__ + "'")


        # Role Assignment
        ################################################################################
        try:
            assignments = role_assignment.assign_roles(self.robots, self.role_requirements())
        except role_assignment.ImpossibleAssignmentError as e:
            logging.error("Unable to satisfy role assignment constraints.  Dropping and temp. blacklisting current play...")
            self.drop_current_play(temporarily_blacklist=True)
        else:
            self.assign_roles(assignments)



    def handle_subbehavior_exception(self, name, exception):
        if name == 'goalie':
            logging.error("Goalie encountered an exception: " + str(exception) + ".  Reloading goalie behavior")
            self.drop_goalie_behavior()
        else:
            logging.error("Play encountered an exception: " + str(exception) + ".  Dropping and temp. blacklisting current play...")
            self.drop_current_play(temporarily_blacklist=True)


    # this is used to force a reselection of a play
    def drop_current_play(self, temporarily_blacklist=False):
        self.temporarily_blacklisted_play_class = self.play.__class__
        self.play = None


    # this is called when the goalie behavior must be reloaded (for example when the goalie.py file is modified)
    def drop_goalie_behavior(self):
        if self.has_subbehavior_with_name('goalie'):
            self.remove_subbehavior('goalie')
        self.setup_goalie_if_needed()


    @property
    def play(self):
        return self._play
    @play.setter
    def play(self, value):
        # trash old play
        if self.play != None:
            self.remove_subbehavior('play')
            self._play = None

        if value != None:
            self._play = value
            self.add_subbehavior(value, name='play', required=True)

        # change notification so ui can update if necessary
        self.play_changed.emit(self.play.__class__.__name__ if self._play != None else "(No Play)")


    # the c++ GameplayModule reaches through the language portal and sets this
    # note that in c++, a value of -1 indicates no assigned goalie, in python we represent the same thing with None
    @property
    def goalie_id(self):
        return self._goalie_id
    @goalie_id.setter
    def goalie_id(self, value):
        self._goalie_id = None if value == -1 else value
        self.setup_goalie_if_needed()
        logging.info("goalie_id set to: " + str(self._goalie_id))


    def setup_goalie_if_needed(self):
        if self.goalie_id == None:
            if self.has_subbehavior_with_name('goalie'):
                self.remove_subbehavior('goalie')
        else:
            if self.has_subbehavior_with_name('goalie'):
                goalie = self.subbehavior_with_name('goalie')
            else:
                goalie = tactics.positions.goalie.Goalie()
                # FIXME: add goalie with high priority?
                self.add_subbehavior(goalie, 'goalie', required=True)

            goalie.shell_id = self.goalie_id


    @property
    def robots(self):
        return self._robots
    @robots.setter
    def robots(self, robots):
        self._robots = robots if robots != None else []
