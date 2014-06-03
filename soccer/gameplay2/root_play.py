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
        print("RootPlay running...")
        if self.play != None:
            print(str(self.play))
            self.play.run()


    def on_exit_running(self):
        self._play = None


    @property
    def play(self):
        return self._play


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
