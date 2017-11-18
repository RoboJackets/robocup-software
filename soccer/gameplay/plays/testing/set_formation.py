import play
import behavior
import tactics.set_formation
import robocup
import constants


class SetFormation(play.Play):
    def __init__(self):
        super().__init__(continuous=False)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')
        self.add_transition(
            behavior.Behavior.State.running, behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name("SetFormation").state == behavior.Behavior.State.completed,
            'all robots reach target positions')
        self.add_transition(
            behavior.Behavior.State.completed, behavior.Behavior.State.running,
            lambda: self.subbehavior_with_name("SetFormation").state == behavior.Behavior.State.running,
            'robots havent reached target positions')

        l = tactics.set_formation.SetFormation()
        self.add_subbehavior(l, name="SetFormation", required=True)
