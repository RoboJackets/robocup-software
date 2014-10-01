import play
import behavior
import skills.move
import robocup
import constants

class TeeUp(play.Play):

    def __init__(self):
        super().__init__(continuous=False)

        # Setup transitions
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
            'robots arent teed up')

        # Define the positoins for the T formation
        field_width = constants.Field.Width
        field_length = constants.Field.Length
        x_pos = [-field_width/4, 0, field_width/4, 0, 0, 0]
        y_pos = [field_length/2-0.5, field_length/2-0.5, field_length/2-0.5,
                field_length/2-1, field_length/2-1.5, field_length/2-2]

        # Add subbehaviors that make the robots move
        for i in range(6):
            pt = robocup.Point(x_pos[i], y_pos[i])
            self.add_subbehavior(skills.move.Move(pt),
                name="robot" + str(i),
                required=False,
                priority=6 - i)

    def all_subbehaviors_completed(self):
        return all([b.behavior_state == behavior.Behavior.State.completed 
                or b.robot == None for b in self.all_subbehaviors()])