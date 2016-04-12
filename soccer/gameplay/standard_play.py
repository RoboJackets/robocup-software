import play
import ui
import tactics


## @brief A standardized play that handles actions that an average play needs
# Right now, this is only used to implement a standard way to run or not run
# the play with defense, but any action that a normal play should do can be 
# placed here
class StandardPlay(play.Play):

    #Performs actions that all "Standard Plays" should do on initialization
    #Note: This method is called many times during the duration of a play,
    #Not just on selection
    def __init__(self, continuous):
        super().__init__(continuous)
        self.use_standard_defense()

    #If the "Use Defense" checkbox is checked and the play isn't already running
    #defense, then it adds the defense behavior. If the box isn't checked and the
    #play is running defense then it removes the behavior. Also note: it ignores
    #the requirement for goalie if the box is checked.
    def use_standard_defense(self):
        if ui.main.defenseEnabled() and not self.has_subbehavior_with_name(
                'defense'):
            self.add_subbehavior(tactics.defense.Defense(),
                                 'defense',
                                 required=False)

        elif not ui.main.defenseEnabled():
            if self.has_subbehavior_with_name('defense'):
                self.remove_subbehavior('defense')

    #If using defense, it defaults to play's method, otherwise returns True
    @classmethod
    def handles_goalie(cls):
        if ui.main.defenseEnabled():
            return True
        else:
            return super().handles_goalie()
