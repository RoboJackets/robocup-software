import main
import robocup
import behavior
import constants
import enum
import random

import standard_play
import evaluation.ball
import evaluation.passing_positioning
import tactics.coordinated_pass
import skills.move
import skills.capture
import tactics.positions.goalie



class StaticFormation(standard_play.StandardPlay):

    FORMATION_FULLBACK = 2
    FORMATION_MIDFIELD = 1
    FORMATION_STRIKER  = 2

    FORMATION_FULLBACK_SQUEEZE = 0.5
    FORMATION_MIDFIELD_SQUEEZE = 1
    FORMATION_STRIKER_SQUEEZE = 1

    class State(enum.Enum):
        # We don't have the ball
        # do some stuff here
        defense = 1

        # Passes around the formation
        # making slow progress forward
        passing = 2

        # A defender has the ball
        # Passes to the midfielder
        # moves up the outside of the field
        # Recieves a pass from the midfielder
        # (The midfielder then takes over the defenders spot)
        overtake = 3

        # Move the ball to the other side of the field
        # without going through one of the center robots
        switch = 4

        # Chip the ball across the opponent goal towards a few of
        # our strikers / midfielder
        cross = 5

        # Through pass between two opponent defenders
        # and let one of our robots move onto the pass
        through = 6

        # Shoot on goal
        shoot = 7

        # Default state to transition to all the others
        formation = 8


    def __init__(self):
        super().__init__(continuous=False)

        for s in StaticFormation.State:
            self.add_state(s, behavior.Behavior.State.running)

        self.width = constants.Field.Width
        self.length = constants.Field.Length

        self.formation_width = self.width * .8
        self.formation_length = self.length / 3
        
        self.formation_center = main.ball().pos - robocup.Point(0, 0.1)


        self.fullback_moves = []
        self.midfield_moves = []
        self.striker_moves = []

        # This is really the goalie, but naming it defense allows us not to have the defense automatically be added
        self.add_subbehavior(skills.move.Move(robocup.Point(0,0)), "defense", required=True, priority=100)

        self.add_transition(behavior.Behavior.State.start,
                            StaticFormation.State.formation, lambda: True,
                            'immediately')

        self.add_transition(StaticFormation.State.formation,
                            StaticFormation.State.passing,
                            lambda: True,
                            'passing')

    def on_enter_running(self):
        self.fullback_x_locs = self.get_horizontal_locations(StaticFormation.FORMATION_FULLBACK_SQUEEZE, StaticFormation.FORMATION_FULLBACK)
        self.midfield_x_locs = self.get_horizontal_locations(StaticFormation.FORMATION_MIDFIELD_SQUEEZE, StaticFormation.FORMATION_MIDFIELD)
        self.striker_x_locs  = self.get_horizontal_locations(StaticFormation.FORMATION_STRIKER_SQUEEZE,   StaticFormation.FORMATION_STRIKER)

        self.set_location_movement(self.fullback_moves, StaticFormation.FORMATION_FULLBACK, self.fullback_x_locs, -self.formation_length/2, "fullback", [100]*StaticFormation.FORMATION_FULLBACK)
        self.set_location_movement(self.midfield_moves, StaticFormation.FORMATION_MIDFIELD, self.midfield_x_locs,                        0, "midfield", [100]*StaticFormation.FORMATION_MIDFIELD)
        self.set_location_movement(self.striker_moves,  StaticFormation.FORMATION_STRIKER,  self.striker_x_locs,   self.formation_length/2, "striker",  [100]*StaticFormation.FORMATION_STRIKER)

    def execute_running(self):
        self.formation_center = self.get_formation_center()

        self.set_location_movement(self.fullback_moves, StaticFormation.FORMATION_FULLBACK, self.fullback_x_locs, -self.formation_length/2, "fullback", [100]*StaticFormation.FORMATION_FULLBACK)
        self.set_location_movement(self.midfield_moves, StaticFormation.FORMATION_MIDFIELD, self.midfield_x_locs,                        0, "midfield", [100]*StaticFormation.FORMATION_MIDFIELD)
        self.set_location_movement(self.striker_moves,  StaticFormation.FORMATION_STRIKER,  self.striker_x_locs,   self.formation_length/2, "striker",  [100]*StaticFormation.FORMATION_STRIKER)

    def get_horizontal_locations(self, squeeze, num_robots):
        if (num_robots == 1):
            return [0]
        offset = self.formation_width / (num_robots - 1)

        locs = []
        for i in range(num_robots):
            locs.append(squeeze*(-self.formation_width/2 + i*offset))

        return locs

    def set_location_movement(self, move_reference_list, num_robots, x_locs, y_loc, name_base, priority_list):
        # if it hasn't been initialized yet, create all the moves
        if (len(move_reference_list) == 0):
            for i in range(num_robots):
                move_reference_list.append(
                    skills.move.Move(
                        robocup.Point(self.formation_center.x + x_locs[i], self.formation_center.y + y_loc)
                    )
                )

                self.add_subbehavior(move_reference_list[i], name_base + str(i), required=True, priority=priority_list[i])

            print("Created list")
        # List has already been created, just change target on moves
        else:
            for i in range(num_robots):
                move_reference_list[i].pos = robocup.Point(self.formation_center.x + x_locs[i], self.formation_center.y + y_loc)

    def get_formation_center(self):
        return main.ball().pos - robocup.Point(0, 0.1)

    def on_enter_passing(self):
        # Get pass target
        # Pass to them
        # Use current robot nearest to ball
        pass_pt = self.get_closest_robot_to_ball()

        possible_targets = self.get_possible_pass_targets(pass_pt)

        print(possible_targets)
        rand_target = random.choice(possible_targets)

        self.passer = self.get_move_at_pos(pass_pt)
        self.receiver = self.get_move_at_pos(rand_target)

        self.subbehavior_info_with_behavior(self.passer)['required'] = False
        self.subbehavior_info_with_behavior(self.receiver)['required'] = False

        self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(rand_target), "passing", required=True, priority=10)

    def get_possible_pass_targets(self, from_robot_pos):
        # Get all the robots within 1/2 the max formation distance
        # From forwards to back

        # There's a better way to do this that we should look into
        max_dist = robocup.Point(self.formation_width, self.formation_length).mag()/2
        min_dist = 0.01

        valid_targets = []

        for m in self.striker_moves:
            dist = (from_robot_pos - m.pos).mag()
            if (dist < max_dist and dist > min_dist):
                valid_targets.append(m.pos)

        for m in self.midfield_moves:
            dist = (from_robot_pos - m.pos).mag()
            if (dist < max_dist and dist > min_dist):
                valid_targets.append(m.pos)
        
        for m in self.fullback_moves:
            dist = (from_robot_pos - m.pos).mag()
            if (dist < max_dist and dist > min_dist):
                valid_targets.append(m.pos)

        return valid_targets

    def get_closest_robot_to_ball(self):
        min_dist = float('inf')
        min_pos = None

        for m in self.striker_moves:
            dist = (main.ball().pos - m.pos).mag()
            if (dist < min_dist):
                min_pos = m.pos
                min_dist = dist


        for m in self.midfield_moves:
            dist = (main.ball().pos - m.pos).mag()
            if (dist < min_dist):
                min_pos = m.pos
                min_dist = dist

        
        for m in self.fullback_moves:
            dist = (main.ball().pos - m.pos).mag()
            if (dist < min_dist):
                min_pos = m.pos
                min_dist = dist

        if (min_pos is None):
            print('No closest robot')

        return min_pos

    def get_move_at_pos(self, pos):
        for m in self.striker_moves:
            dist = (pos - m.pos).mag()
            if (dist < 0.01):
                return m

        for m in self.midfield_moves:
            dist = (pos - m.pos).mag()
            if (dist < 0.01):
                return m
        
        for m in self.fullback_moves:
            dist = (pos - m.pos).mag()
            if (dist < 0.01):
                return m

    @classmethod
    def score(cls):
        if (not main.game_state().is_playing()):
            return float("inf")
        return 10