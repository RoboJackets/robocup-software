import play
import ui.main
import tactics
import main
import situational_play_selection
from typing import List

## @brief A standardized play that handles actions that an average play needs
# Right now, this is only used to implement a standard way to run or not run
# the play with defense, but any action that a normal play should do can be 
# placed here
class StandardPlay(play.Play):

    ##
    # Performs actions that all "Standard Plays" should do on initialization
    # Note: This method is called many times during the duration of a play,
    # Not just on selection
    def __init__(self, continuous):
        super().__init__(continuous)
        self.use_standard_defense()

        #If the "Use Defense" checkbox is checked and the play isn't already running
        #defense, then it adds the defense behavior. If the box isn't checked and the
        #play is running defense then it removes the behavior. Also note: it ignores
        #the requirement for goalie if the box is checked.

    ##A private list of situations that are applicable to this play

    _situationList = list()  # type: List[situational_play_selection.SituationalPlaySelector.Situation]

    def use_standard_defense(self):
        if ui.main.defenseEnabled() and not self.has_subbehavior_with_name(
                'defense'):
            self.add_subbehavior(tactics.defense.Defense(),
                                 'defense',
                                 required=False)
        elif not ui.main.defenseEnabled():
            if self.has_subbehavior_with_name('defense'):
                self.remove_subbehavior('defense')

    ## 
    # Handles activity while the play is active. A play wishing to utilize this
    # method in additionto having an "execute_running" method of its own must call
    # it via super
    def execute_running(self):
        self.use_standard_defense()

    #Since the standard_play handles defense, it will always handle the goalie
    @classmethod
    def handles_goalie(cls):
        return True

    ##
    # Call to attempt to preempt the play
    # Returns true if the preempt is successful 
    # Override if you want more complex responce to being preempted
    def try_preempt(self):
        self.terminate()
        return True

    ##
    # Returns true if this play is valid for the passed situation
    #
    @classmethod
    def is_valid(cls, situation):
        return situation in cls._situationList

    ##
    # The score function for standard play will check if situation analysis
    # is enabled, will return a base score based on situation match if it is
    # or return float('inf') if it is not
    #
    # The expected interaction is to override this function can call super()
    # to get a baseline score, then to modify that score based on situational
    # factors. If you get a float("inf") back you know that situation analysis
    # is not running and you will need to overwrite that with some other score
    #
    @classmethod
    def score(cls):
        if (not main.situationAnalysis.enabled):
            return float('inf')
        else:
            if (cls.is_valid(main.situationAnalysis.getSituation())):
                return main.situationAnalysis.inSituationScore
            else:
                return main.situationAnalysis.outSituationScore
