import main
import robocup
import enum
import constants
import composite_behavior
import behavior
import formation.positions.position
import formation.positions.simple_goalie
import formation.positions.simple_position

# Create the formation
# Position for every robot
# In their prominant location
# Positions will take influence areas
# that they should avoid
# All robots are idle
# None are required
# Should keep robots in position unless they are taken
# Create zones for each robot
# Move formation up and down field


# Controls the formation
# Deals with...
#  setting up the formation and the positions it contains
#  passing influence zones down to positions
#  moving the formation around
#  removing and adding robots to the formation when plays want them
class Controller(composite_behavior.CompositeBehavior):
    FORMATION_WIDTH  = 0.8 # % of field width
    FORMATION_LENGTH = 0.7 # % of field length

    STRIKER_Y = 1
    MIDFIELDER_Y = 0
    DEFENDER_Y = -1

    CENTER_OFFSET = -.25

    def __init__(self):
        super().__init__(continuous=True)
        ### Formation specifics ###

        # [Their  Goal] #
        #   X       X   #
        #       X       #
        #     X   X     #
        #               #
        #       X       #
        # [Our    Goal] #

        # Generically create the position classes
        # When new positions behaviors are created
        # Replace the next lines with the target class
        # Type should remain the same unless the general
        #  of the formation changes (AKA 3 defenders now etc)
        self.Striker_Left  = formation.positions.simple_position.SimplePosition(formation.positions.position.Position.Type.Striker,
                                               "Left Striker")
        self.Striker_Right = formation.positions.simple_position.SimplePosition(formation.positions.position.Position.Type.Striker,
                                               "Right Striker")

        self.Midfielder_Center = formation.positions.simple_position.SimplePosition(formation.positions.position.Position.Type.Midfielder,
                                                   "Center Midfielder")
        
        self.Defender_Left  = formation.positions.simple_position.SimplePosition(formation.positions.position.Position.Type.Defender,
                                                "Left Defender")
        self.Defender_Right = formation.positions.simple_position.SimplePosition(formation.positions.position.Position.Type.Defender,
                                                "Right Defender")

        self.Goalie = formation.positions.simple_goalie.SimpleGoalie(formation.positions.position.Position.Type.Goalie,
                                        "Goalie")

        self.positions = [self.Striker_Left, self.Striker_Right,
                          self.Midfielder_Center,
                          self.Defender_Left, self.Defender_Right]

        # Location inside the formation relatively
        # Formation defined by relative coordinates
        # Most extreme positions define +- 1 for X and Y
        # self.Goalie is not included in position definitions
        self.Striker_Left.relative_pos  = robocup.Point(-1, Controller.STRIKER_Y)
        self.Striker_Right.relative_pos = robocup.Point( 1, Controller.STRIKER_Y)

        self.Midfielder_Center.relative_pos = robocup.Point(0, Controller.MIDFIELDER_Y)

        self.Defender_Left.relative_pos  = robocup.Point(-0.5, Controller.DEFENDER_Y)
        self.Defender_Right.relative_pos = robocup.Point( 0.5, Controller.DEFENDER_Y)

        self.Goalie.relative_pos = None

        # Setup normal pass options
        # These are which positions that are standard pass options
        # It is normal to pass within "triangles" near your position
        # This solidifies it into code
        # Only positions in this list should be getting open for passes
        #  when said position has the ball
        self.Striker_Left.pass_options  = [self.Striker_Right, self.Midfielder_Center, self.Defender_Left]
        self.Striker_Right.pass_options = [self.Striker_Left, self.Midfielder_Center, self.Defender_Right]

        self.Midfielder_Center.pass_options = [self.Striker_Left, self.Striker_Right, self.Defender_Left, self.Defender_Right]

        self.Defender_Left.pass_options  = [self.Striker_Left, self.Midfielder_Center, self.Defender_Right]
        self.Defender_Right.pass_options = [self.Striker_Right, self.Midfielder_Center, self.Defender_Left]


        self.formation_width = Controller.FORMATION_WIDTH * constants.Field.Width
        self.formation_length = Controller.FORMATION_LENGTH * constants.Field.Length
        self.formation_center = main.ball().pos

        self.clip_formation_center()
        self.update_formation_center()
        self.update_formation_location()

        for p in self.positions:
            IDLE = 1
            self.add_subbehavior(p, p.str_name, required=False, priority=IDLE)

        # Just jump to running instantly
        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running,
                            lambda: True, "starting")

    # Clip formation such that all robots are in bounds
    def clip_formation_center(self):
        self.formation_center.y = max(self.formation_center.y,
                                      self.formation_length/2 + constants.Field.PenaltyShortDist)
        self.formation_center.y = min(self.formation_center.y,
                                      constants.Field.PenaltyShortDist - self.formation_length/2)
        
        self.formation_center.x = max(self.formation_center.x,
                                      -constants.Field.Width + self.formation_width/2)
        self.formation_center.x = min(self.formation_center.x,
                                      constants.Field.Width - self.formation_width/2)

    # Move center of formation to correct location
    def update_formation_center(self):
        self.formation_center = robocup.Point(0,4.5)
        return
        # Shift formation such that it "falls" about 25% ahead of each of the
        # 3 main lines (striker, midfielder, defender) y values

        # Given that the ball is at the following letter positions
        # The following is the resulting line we are trying to be behind
        # Striker    -------------------
        #               A
        #
        #                       B
        # Midfielder -------------------
        #               C
        #                       D
        #
        # Defender   -------------------
        #                   E
        #
        # A: Shift so striker line is just in front of A
        # B: Shift so midfielder line is just in front of B
        # C: Shift so midfielder line is just in front of C
        # D: Use what ever line was previously chosen (Stops oscillation)
        # E: Shift so defender line is just in front of E

        # If center only `shift_down_bound` % behind line, just move line backwards
        #  a little
        # If center `shift_up_bound` % behind line, move up to the next line
        #
        shift_down_bound = .1 # m
        shift_up_bound = .15 # m

        possible_lines = [Controller.STRIKER_Y, Controller.MIDFIELDER_Y, Controller.DEFENDER_Y]

        target_line = None
        real_target = 0
        for l in possible_lines:
            real_target = self.formation_center.y + (l + Controller.CENTER_OFFSET)*self.formation_length/2
            
            target_line = l

            # If the line is almost at correct position
            if (real_target - shift_down_bound < main.ball().pos.y):
                break
            # No man's zone so use previous line
            # This should always be the current line or the one after
            elif (real_target - shift_up_bound < main.ball().pos.y):
                target_line = self.previous_target_line


        alpha = .9

        # Set Y of formation to that line
        # Set X to average towards ball position
        self.formation_center.y = real_target
        self.formation_center.x = alpha*self.formation_center.x + (1 - alpha)*main.ball().pos.x
        self.previous_target_line = target_line

        self.clip_formation_center()

    # Update target positions given the formation center
    # Assumes formation center is valid
    #  AKA Not out of bounds
    def update_formation_location(self):
        for p in self.positions:
            p.target_pos =  self.formation_center + \
                            p.relative_pos * robocup.Point(self.formation_width / 2,
                                                           self.formation_length / 2)