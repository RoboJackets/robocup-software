import play
import ui
import tactics

class StandardPlay(play.Play):
    def __init__(self, continuous):
        super().__init__(continuous)

    def use_standard_defense(self):
        if ui.main.defenseEnabled() and (not self.has_subbehavior_with_name('defense')):
            self.add_subbehavior(tactics.defense.Defense(), 'defense', required=False)
        elif self.has_subbehavior_with_name('defense'):
            self.remove_subbehavior('defense')

    def execute_running(self):
        self.use_standard_defense()


