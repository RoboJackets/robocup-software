import play
import behavior
import skills.move
import robocup


class LineUp(play.Play):

    def __init__(self):
        super().__init__(continuous=False)

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')
        self.add_transition(behavior.Behavior.State.running,
            behavior.Behavior.State.completed,
            lambda: self.all_subbehaviors_completed(),
            'all robots reach target positions')
        self.add_transition(behavior.Behavior.State.completed,
            behavior.Behavior.State.running,
            lambda: not self.all_subbehaviors_completed(),
            'robots arent lined up')

        # add subbehaviors for all robots, instructing them to line up
        start_x = -1.0
        start_y = 0.5
        spacing_y = 0.25
        for i in range(6):
            pt = robocup.Point(start_x, start_y + i * spacing_y)
            self.add_subbehavior(skills.move.Move(pt),
                name="robot" + str(i),
                required=False,
                priority=6 - i)


    def all_subbehaviors_completed(self):
        return all([b.behavior_state == behavior.Behavior.State.completed or b.robot == None for b in self.all_subbehaviors()])
