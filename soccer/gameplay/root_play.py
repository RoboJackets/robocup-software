from typing import Tuple, List, Type, Optional
from play import Play
import robocup
from behavior import Behavior
import plays.stopped
import plays.testing.test_coach
import logging
from PyQt5 import QtCore
import main
import evaluation.double_touch
import tactics.positions.goalie
import role_assignment
from role_assignment import AssignedRoleReqTree
import traceback


## The RootPlay is basically the python-side of the c++ GameplayModule
# it coordinates the selection of the 'actual' play and handles the goalie behavior
class RootPlay(Play, QtCore.QObject):
    def __init__(self):
        QtCore.QObject.__init__(self)
        Play.__init__(self, continuous=True)
        self._play: Optional[Play] = None
        self.add_transition(Behavior.State.start, Behavior.State.running,
                            lambda: True, 'immediately')

        # if a play fails for some reason, we can temporarily blacklist it, which removes it from play
        # selection for the next iteration, then enables it again
        self.temporarily_blacklisted_play_class: Optional[Type[Play]] = None
        self._currently_restarting = False
        self._robots: List[robocup.Robot] = []

    play_changed = QtCore.pyqtSignal("QString")

    def execute_running(self):
        # update double touch tracker
        evaluation.double_touch.tracker().spin()

        # cache and calculate the score() function for each play class
        main.play_registry().recalculate_scores()

        # Play Selection
        ################################################################################

        if main.game_state().is_stopped():
            evaluation.double_touch.tracker().restart()
            if main.game_state().is_placement():
                if not isinstance(self.play,
                                  plays.restarts.placement.Placement):
                    logging.info("Placing Ball")
                    self.play = plays.restarts.placement.Placement()
                    self._currently_restarting = True
            else:
                if self.play is None or not self.play.run_during_stopped():
                    logging.info(
                        "Running 'Stopped' play due to game state change")
                    self.play = plays.stopped.Stopped()
                    self._currently_restarting = True
        elif main.game_state().is_halted():
            evaluation.double_touch.tracker().restart()
            self.play = None
        else:
            # (play_class, score value) tuples
            enabled_plays_and_scores: List[Tuple[
                Type[Play],
                float]] = main.play_registry().get_enabled_plays_and_scores()

            # only let restart play run once
            enabled_plays_and_scores = [
                p
                for p in enabled_plays_and_scores
                if not p[0].is_restart() or (p[0].is_restart() and
                                             self._currently_restarting)
            ]

            # handle temporary blacklisting
            # we remove the blacklisted play class from selection for this iteration, then unblacklist it
            enabled_plays_and_scores = [
                p
                for p in enabled_plays_and_scores
                if p[0] != self.temporarily_blacklisted_play_class
            ]
            self.temporarily_blacklisted_play_class = None

            # see if we need to kill current play or if it's done running
            if self.play is not None:
                if self.play.__class__ not in map(lambda tup: tup[0],
                                                  enabled_plays_and_scores):
                    logging.info("Current play '" +
                                 self.play.__class__.__name__ +
                                 "' no longer enabled, aborting")
                    self.play.terminate()
                    self.play = None
                elif self.play.is_done_running():
                    logging.info("Current play '" +
                                 self.play.__class__.__name__ +
                                 "' finished running")
                    if self.play.is_restart:
                        self._currently_restarting = False
                    self.play = None
                elif self.play.__class__.score() == float("inf"):
                    logging.info("Current play '" +
                                 self.play.__class__.__name__ +
                                 "' no longer applicable, ending")
                    self.play.terminate()
                    self.play = None

            if self.play is None:
                try:
                    if len(enabled_plays_and_scores) > 0:
                        # select the play with the smallest value for score()
                        play_class_and_score = min(enabled_plays_and_scores,
                                                   key=lambda tup: tup[1])

                        # run the play with the lowest score, as long as it isn't inf
                        if play_class_and_score[1] != float("inf"):
                            play_class = play_class_and_score[0]
                            self.play = play_class()  # instantiate it
                    else:
                        # there's no available plays to run
                        pass
                except Exception as e:
                    logging.error("Exception occurred during play selection: "
                                  + str(e))
                    traceback.print_exc()
                if self.play is not None:
                    logging.info("Chose new play: '" +
                                 self.play.__class__.__name__ + "'")

        # Role Assignment
        ################################################################################
        try:
            assignments: AssignedRoleReqTree = role_assignment.assign_roles(
                self.robots, self.role_requirements())
        except role_assignment.ImpossibleAssignmentError as e:
            logging.error(
                "Unable to satisfy role assignment constraints.  Dropping and temp. blacklisting current play...")
            self.drop_current_play(temporarily_blacklist=True)
        else:
            self.assign_roles(assignments)

    def handle_subbehavior_exception(self, name, exception):
        if name == 'goalie':
            logging.error("Goalie encountered an exception: " + str(exception)
                          + ".  Reloading goalie behavior")
            traceback.print_exc()
            self.drop_goalie_behavior()
        else:
            logging.error("Play '" + self.play.__class__.__name__ +
                          "' encountered an exception: " + str(exception) +
                          ".  Dropping and temp. blacklisting current play...")
            traceback.print_exc()
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
    def play(self) -> Optional[Play]:
        return self._play

    @play.setter
    def play(self, value):
        # trash old play
        if self.play is not None:
            self.remove_subbehavior('play')
            self._play = None

        if value is not None:
            self._play = value

            # see if this play handles the goalie by itself
            if value.__class__.handles_goalie():
                self.drop_goalie_behavior()

            self.add_subbehavior(value, name='play', required=True)

        # make sure somebody handles the goalie
        self.setup_goalie_if_needed()

        # change notification so ui can update if necessary
        self.play_changed.emit(self.play.__class__.__name__ if self.
                               _play is not None else "(No Play)")

    ## the c++ GameplayModule reaches through the language portal and sets this
    # note that in c++, a value of -1 indicates no assigned goalie, in python we represent the same thing with None
    @property
    def goalie_id(self):
        goalie = main.context().game_state.get_goalie_id()
        return None if goalie == -1 else goalie

    def setup_goalie_if_needed(self):
        if self.goalie_id is None:
            if self.has_subbehavior_with_name('goalie'):
                self.remove_subbehavior('goalie')
        else:
            if self.has_subbehavior_with_name('goalie'):
                goalie = self.subbehavior_with_name('goalie')
            elif self.play is None or not self.play.__class__.handles_goalie():
                goalie = tactics.positions.goalie.Goalie()
                self.add_subbehavior(goalie, 'goalie', required=True)
            else:
                goalie = None

            if goalie is not None:
                goalie.shell_id = self.goalie_id

    @property
    def robots(self):
        return self._robots

    @robots.setter
    def robots(self, robots: Optional[List[robocup.OurRobot]]):
        self._robots = robots if robots is not None else []

    def __str__(self):
        return '\n'.join([str(bhvr) for bhvr in self.all_subbehaviors()])
