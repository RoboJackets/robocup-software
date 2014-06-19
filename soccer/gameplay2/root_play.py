from play import *
from behavior import *
from plays.line_up import *
import logging


# the RootPlay is basically the python-side of the c++ GameplayModule
# it coordinates the selection of the 'actual' play and handles the goalie behavior
class RootPlay(Play):

    def __init__(self):
        super().__init__(continuous=True)
        self._play = None

        self.add_transition(Behavior.State.start, Behavior.State.running, lambda: True, 'immediately')


    def on_enter_running(self):
        self._play = LineUp()


    def execute_running(self):
        # TODO: do goalie stuff
        if self.play == None:
            # select the play with the largest value for score()
            self.play = max(main.play_registry().get_enabled_plays(), key=lambda p: p.score())
        if self.play != None:
            try:
                self.play.run()
            except Error as e:
                logging.error("Play '" + self.play.__class__.__name__ + "' encountered exception: " + str(e) + ". aborting and reselecting play...")


    def on_exit_running(self):
        self._play = None


    # this is used to force a reselection of a play
    def drop_current_play(self):
        self.play = None


    # this is called when the goalie behavior must be reloaded (for example when the goalie.py file is modified)
    def drop_goalie_behavior(self):
        self._goalie_behavior = None


    @property
    def play(self):
        return self._play
    @play.setter
    def play(self, value):
        self._play = value
        # TODO: update play label in gui


    @property
    def goalie_behavior(self):
        return self._goalie_behavior


    @Play.robots.setter
    def robots(self, robots):
        # FIXME: assign goalie

        #FIXME: call superclass setter?
        self._robots = robots

        print("root play set robots: " + str(robots))
        # for r in robots:
        #     print("\trobot: " + str(r))

        # pass robots to play
        if self.play != None:
            self.play.robots = robots
