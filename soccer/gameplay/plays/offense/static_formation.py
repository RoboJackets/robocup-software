import main
import robocup
import behavior
import constants
import enum
import random
import numpy as np

import standard_play
import evaluation.ball
import evaluation.passing_positioning
import tactics.coordinated_pass
import skills.move
import skills.capture
from tactics.positions.goalie import Goalie

def copy_function(f, name=None):
    return types.FunctionType(f.func_code, f.func_globals, name or f.func_name,
                f.func_defaults, f.func_closure)


class StaticFormation(standard_play.StandardPlay):
    # Contains the target robot position, move subehavior
    # As well as all the pas targets from this position
    #
    # TODO: Think about just returning the controller robot position instead of the target robot position
    # Or just when doing which robot has ball, use the real robot positions
    class FormationPosition():


        def __init__(self, relative_pos, name, formation_center, formation_half_width, formation_half_length):
            # Relative position inside formation
            class PositionMove(skills.move.Move):
                pass

            self.relative_pos = relative_pos

            self.formation_center = formation_center
            self.formation_half_width = formation_half_width
            self.formation_half_length = formation_half_length

            self.update_target_pos()

            # Other positions that we should pass to
            self.pass_positions = []

            # All the stuff needed for pass behaviors
            self.name = name
            self.priority = 10
            self.required = True
            self.move_skill = PositionMove
            self.move_skill.__name__ = 'Move_{}'.format(name)
            self.move_behavior = self.move_skill(self.target_pos)

        def update_target_pos(self):
            offset_in_formation = robocup.Point(self.relative_pos.x * self.formation_half_width,
                                                self.relative_pos.y * self.formation_half_length)
            self.target_pos = self.formation_center + offset_in_formation

        def update_move_target(self):
            self.move_behavior.pos = self.target_pos

        def take_control(self):
            self.required = False
            self.priority = 100

        def release_control(self):
            self.required = True
            self.priority = 10

    #
    # Each of these states are a different "play" that can be created
    # They all have very specific actions that are going to be taken
    # Once each is done, it'll go back to the main formation type thing for another
    # Action to be taken
    # Any robots not needed in the play stay in the formation positions
    # 

    class State(enum.Enum):
        # We don't have the ball
        # do some stuff here
        defense = 1

        # Passes around the formation
        # making slow progress forward
        passing = 2

        # A defender (or midfielder) has the ball
        # Passes to the midfielder (or striker)
        # moves up the outside of the field
        # Receives a pass from the midfielder (or striker)
        overtake = 3
        overtake_second_pass = 4

        # Move the ball to the other side of the field
        # without going through one of the center robots
        switch = 5

        # Chip the ball across the opponent goal towards a few of
        # our strikers / midfielder
        cross = 6
        cross_into_middle = 7

        # Shoot on goal
        shoot = 8

        # Default state to transition to all the others
        formation = 9


    def __init__(self):
        super().__init__(continuous=True)

        for s in StaticFormation.State:
            self.add_state(s, behavior.Behavior.State.running)


        # This is really the goalie, but naming it defense allows us not to have the defense automatically be added
        self.remove_all_subbehaviors()
        #self.add_subbehavior(skills.move.Move(robocup.Point(0,0)), 'defense', required=True, priority=100)
        self.goalie = Goalie()
        self.goalie.shell_id = main.system_state().game_state.get_goalie_id()
        self.add_subbehavior(self.goalie, "Goalie", required=True, priority=100)

        self.width = constants.Field.Width
        self.length = constants.Field.Length

        self.formation_width = self.width * .6
        self.formation_length = self.length * .3

        # Forces at least one frame between starting the other play stuff
        self.ctr = 0

        # Random number that selects which play to run
        # Rand num is between 0 and 99
        # The score we comapre against represents probability of being chosen
        # For simplicity, the plays are chosen in reverse order of declaration
        self.rand_num = 0

        # Center location of the defense        
        self.formation_center = robocup.Point(0, constants.Field.Length/2) - robocup.Point(0, 0.3)

        # List of the positions we have in this formation
        self.strikers = []
        self.midfielders = []
        self.fullbacks = []
        self.all_positions = []

        self.create_formation()

        # Special args to the robot selection function for each play
        self.position_with_ball = None


        ################################################################################
        ######                                                                    ######
        ######                       Private Play Variables                       ######
        ######                                                                    ######
        ################################################################################

        # All of these will be within the specific plays
        # I still need to figure out a good way to do this automatically, but this is 
        # a reasonable start

        #######################################
        ####                               ####
        #### Private Variables for passing ####
        ####                               ####
        #######################################
        self.passing_kicker_position = None
        self.passing_receiver_position = None

        ########################################
        ####                                ####
        #### Private Variables for overtake ####
        ####                                ####
        ########################################
        self.overtake_kicker_position = None
        self.overtake_receiver_position = None

        ######################################
        ####                              ####
        #### Private Variables for switch ####
        ####                              ####
        ######################################
        self.switch_kicker_position = None
        self.switch_receiver_position = None

        #####################################
        ####                             ####
        #### Private Variables for cross ####
        ####                             ####
        #####################################
        self.cross_shooter_position = None
        self.cross_crosser_position = None
        self.cross_extra_support_position = None
        self.cross_back_pass_position = None

        #####################################
        ####                             ####
        #### Private Variables for shoot ####
        ####                             ####
        #####################################
        self.shoot_kicker_position = None

        #####################################################################
        ######                                                         ######
        ######                       Transitions                       ######
        ######                                                         ######
        #####################################################################


        self.add_transition(behavior.Behavior.State.start,
                            StaticFormation.State.formation,
                            lambda: True,
                            'immediately')

        #################################
        ####                         ####
        #### Transitions for passing ####
        ####                         ####
        #################################

        self.add_transition(StaticFormation.State.formation,
                            StaticFormation.State.passing,
                            lambda: self.try_choosing_play(self.passing_score_function) and
                                    self.enforce_play_priority_order(StaticFormation.State.passing),
                            'passing')

        self.add_transition(StaticFormation.State.passing,
                            StaticFormation.State.formation,
                            lambda: self.subbehavior_with_name('simple_pass').is_done_running(),
                            'done passing')


        ##################################
        ####                          ####
        #### Transitions for overtake ####
        ####                          ####
        ##################################
        
        self.add_transition(StaticFormation.State.formation,
                            StaticFormation.State.overtake,
                            lambda: self.try_choosing_play(self.overtake_score_function) and
                                    self.enforce_play_priority_order(StaticFormation.State.overtake),
                            'overtake first pass')

        self.add_transition(StaticFormation.State.overtake,
                            StaticFormation.State.overtake_second_pass,
                            lambda: self.subbehavior_with_name('overtake_first_pass').is_done_running(),
                            'overtake second pass')

        self.add_transition(StaticFormation.State.overtake_second_pass,
                            StaticFormation.State.formation,
                            lambda: self.subbehavior_with_name('overtake_second_pass').is_done_running(),
                            'done overtake')


        ################################
        ####                        ####
        #### Transitions for switch ####
        ####                        ####
        ################################
        
        self.add_transition(StaticFormation.State.formation,
                            StaticFormation.State.switch,
                            lambda: self.try_choosing_play(self.switch_score_function) and
                                    self.enforce_play_priority_order(StaticFormation.State.switch),
                            'switch')

        self.add_transition(StaticFormation.State.switch,
                            StaticFormation.State.formation,
                            lambda: self.subbehavior_with_name('switch_pass').is_done_running(),
                            'done switch')

        ###############################
        ####                       ####
        #### Transitions for cross ####
        ####                       ####
        ###############################
        
        self.add_transition(StaticFormation.State.formation,
                            StaticFormation.State.cross,
                            lambda: self.try_choosing_play(self.cross_score_function) and
                                    self.enforce_play_priority_order(StaticFormation.State.cross),
                            'cross setup')

        self.add_transition(StaticFormation.State.cross,
                            StaticFormation.State.cross_into_middle,
                            lambda: self.subbehavior_with_name('cross_setup').is_done_running(),
                            'cross into middle')

        self.add_transition(StaticFormation.State.cross_into_middle,
                            StaticFormation.State.formation,
                            lambda: self.subbehavior_with_name('one_touch').is_done_running(),
                            'done cross')

        ###############################
        ####                       ####
        #### Transitions for shoot ####
        ####                       ####
        ###############################

        self.add_transition(StaticFormation.State.formation,
                            StaticFormation.State.shoot,
                            lambda: self.try_choosing_play(self.shoot_score_function) and
                                    self.enforce_play_priority_order(StaticFormation.State.shoot),
                            'shoot')

        self.add_transition(StaticFormation.State.shoot,
                            StaticFormation.State.formation,
                            lambda: self.subbehavior_with_name('shooter').is_done_running(),
                            'done shooting')


    ###########################################################################
    ######                                                               ######
    ######                       Debug Functions                       ######
    ######                                                               ######
    ###########################################################################

    def draw_actual_center(self):
        positions = [robot.pos for robot in main.system_state().our_robots if robot.shell_id!=self.goalie.shell_id and robot.visible]
        actual_center = robocup.Point(np.mean([pos.x for pos in positions]), np.mean([pos.y for pos in positions]))
        main.system_state().draw_circle(actual_center, .2, (0,0,0),"")
        #print(actual_center)


    ###########################################################################
    ######                                                               ######
    ######                       Scoring Functions                       ######
    ######                                                               ######
    ###########################################################################


    #
    # This will be the standard play score function that is used by the playbook
    # We can change the score to represent how good of situation it is for a single play
    # This below is just a way of adding randomness to the play selection without
    # having to split this out
    #
    # Scores will just be out of 100 right now
    # and if a random number between 0 and 99 is less than
    # the score, it is chosen
    #

    ###########################################
    ####                                   ####
    #### Passing score transition function ####
    ####                                   ####
    ###########################################
    def passing_score_function(self):
        # Always an ok choice to pass around
        # But other choices may be significantly better
        # This is the default choice if nothing else is valid
        return 100

    ############################################
    ####                                    ####
    #### Overtake score transition function ####
    ####                                    ####
    ############################################
    def overtake_score_function(self):
        ball_away_from_their_goal = main.ball().pos.y < self.length * .6
        
        ball_with_midfielders = self.position_with_ball in self.midfielders
        ball_with_fullbacks   = self.position_with_ball in self.fullbacks

        # If we are in a good position, do it 40% of the time, otherwise dont at all
        if (ball_away_from_their_goal and (ball_with_midfielders or ball_with_fullbacks)):
            return 20
        else:
            return 0

    ##########################################
    ####                                  ####
    #### Switch score transition function ####
    ####                                  ####
    ##########################################
    def switch_score_function(self):
        average_opponent_x_loc = 0

        for r in main.their_robots():
            average_opponent_x_loc += r.pos.x

        average_opponent_x_loc /= len(main.their_robots())

        opponents_on_one_side = abs(average_opponent_x_loc) > (self.width/2) * .3
        ball_on_same_side = main.ball().pos.x * average_opponent_x_loc > 0

        # Switch if most of the opponents are on our side with the ball
        # If the ball and x loc are the same sign, multiplying them together 
        # gives a positive number
        if (opponents_on_one_side and ball_on_same_side):
            return 30
        else:
            return 0
        
    #########################################
    ####                                 ####
    #### Cross score transition function ####
    ####                                 ####
    #########################################
    def cross_score_function(self):
        ball_on_their_half = main.ball().pos.y > self.length/2

        # Cross if we are on their half, don't really care about much else for the moment
        if (ball_on_their_half):
            return 40
        else:
            return 0

    #########################################
    ####                                 ####
    #### Shoot score transition function ####
    ####                                 ####
    #########################################
    def shoot_score_function(self):
        ball_near_their_goal = (main.ball().pos - constants.Field.TheirGoalSegment.center()).mag() < 2

        if (ball_near_their_goal):
            return 100
        else:
            return 10

    ###################################################################################
    ######                                                                       ######
    ######                       Individual State Machines                       ######
    ######                                                                       ######
    ###################################################################################


    ######################################
    ####                              ####
    #### Parent Formation Class Stuff ####
    ####                              ####
    ######################################
    def try_choosing_play(self, func):
        return self.rand_num < func()

    def enforce_play_priority_order(self, state):
        return self.ctr > -state.value + 30

    # Create random number so we randomly choose a "play" to do
    def on_enter_formation(self):
        self.rand_num = random.randint(0, 99)

    def execute_running(self):
        self.update_formation_center()
        main.system_state().draw_circle(self.formation_center,.2,(255,255,255), "FORMATION CENTER")
        self.draw_actual_center()
        #print(_game_state.get_goalie_id())
        self.update_formation()
        self.set_special_roles()
        self.update_subbehaviors_settings()

        self.ctr += 1

    def create_formation(self):
        # Create formation specifics
        # TODO: Move this to a config file or something

        # Relative positions of the robots inside the formation
        # 0,0 is the middle of the centerbacks
        # +-1 is furthest left or right robots who should be at edge of formation
        # +1 is furthest forward robot in front of formation
        #
        #
        #
        # +1   ST              ST
        #
        #              MF 
        #
        # -1     FB          FB
        #
        #      -1              +1
        striker_left_rel_pos  = robocup.Point(-1, 1)
        striker_right_rel_pos = robocup.Point( 1, 1)

        midfielder_center_rel_pos = robocup.Point(0, 0)

        fullback_left_rel_pos  = robocup.Point(-.6, -1)
        fullback_right_rel_pos = robocup.Point( .6, -1)

        CreateFormationPosition = lambda rel_pos, name: StaticFormation.FormationPosition(rel_pos, name, self.formation_center, self.formation_width/2, self.formation_length/2)

        # Create the FormationPositions objects
        striker_left  = CreateFormationPosition(striker_left_rel_pos, 'striker_left')
        striker_right = CreateFormationPosition(striker_right_rel_pos, 'striker_right')

        midfielder_center = CreateFormationPosition(midfielder_center_rel_pos, 'midfielder_center')
        
        fullback_left  = CreateFormationPosition(fullback_left_rel_pos, 'fullback_left')
        fullback_right = CreateFormationPosition(fullback_right_rel_pos, 'fullback_right')

        # Create list of all other positions they should pass to normally
        # List is from left to right, front to back
        striker_left.pass_positions  = [striker_right, midfielder_center, fullback_left]
        striker_right.pass_positions = [striker_left, midfielder_center, fullback_right]
        
        midfielder_center.pass_positions = [striker_left, striker_right, fullback_left, fullback_right]

        fullback_left.pass_positions  = [striker_left, midfielder_center, fullback_right]
        fullback_right.pass_positions = [striker_right, midfielder_center, fullback_left]

        # Add them as subbehaviors
        AddPositionSubbehavior = lambda position: self.add_subbehavior(position.move_behavior, position.name, position.required, position.priority)

        AddPositionSubbehavior(striker_left)
        AddPositionSubbehavior(striker_right)

        AddPositionSubbehavior(midfielder_center)

        AddPositionSubbehavior(fullback_left)
        AddPositionSubbehavior(fullback_right)

        # Add to global list
        self.strikers = [striker_left, striker_right]
        self.midfielders = [midfielder_center]
        self.fullbacks = [fullback_left, fullback_right]
        self.all_positions = [striker_left, striker_right, midfielder_center, fullback_left, fullback_right]

    def update_formation(self):
        for position in self.all_positions:
            position.formation_center = self.formation_center
            position.update_target_pos()
            position.update_move_target()

    def update_formation_center(self):
        delta_width = self.width - self.formation_width
        delta_length = self.length - self.formation_length - 2*constants.Field.PenaltyShortDist
        max_center_rect = robocup.Rect(
                            robocup.Point(-delta_width/2 + constants.Robot.Radius, delta_length/2 + constants.Field.PenaltyShortDist),
                            robocup.Point(delta_width/2 - constants.Robot.Radius, self.length - delta_length/2 - constants.Field.PenaltyShortDist))

        main.system_state().draw_segment(robocup.Segment(max_center_rect.get_pt(0), max_center_rect.get_pt(1)), constants.Colors.White, 'FormationMax')

        ball_pos = main.ball().pos

        projected_ball = ball_pos

        # if the ball is outside the movement, project it in
        if (not max_center_rect.contains_point(ball_pos)):
            ball_proj_line = robocup.Segment(ball_pos, self.formation_center)
            intersects = max_center_rect.segment_intersection(ball_proj_line)

            # Get closest edge to center of formation
            if (intersects is not None):
                projected_ball = intersects[0]

        alpha = 0.99
        self.formation_center = self.formation_center * alpha + projected_ball * (1 - alpha)

    def update_subbehaviors_settings(self):
        for position in self.all_positions:
            bhvr_info = self.subbehavior_info_with_behavior(position.move_behavior)
            bhvr_info['required'] = position.required
            bhvr_info['priority'] = lambda: position.priority

    # Specify which robot has the ball
    def set_special_roles(self):
        min_dist = float('inf')
        min_pos = None

        for position in self.all_positions:
            dist_to_ball = (position.target_pos - main.ball().pos).mag()
            if (dist_to_ball < min_dist):
                min_dist = dist_to_ball
                min_pos = position

        self.position_with_ball = min_pos

    #################################
    ####                         ####
    #### Pass Play State Machine ####
    ####                         ####
    #################################

    def on_enter_passing(self):
        # Will be called by parents class
        self.passing_request_robots()

        self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(self.passing_receiver_position.target_pos), 'simple_pass', True, 10)

    # Standardized function rn to select which robots we want
    def passing_request_robots(self):
        # Find place to pass
        # Take position control
        possible_pass_targets = self.position_with_ball.pass_positions

        # Save which two positions we steal
        self.passing_kicker_position = self.position_with_ball

        # Do a weighted random thing to try and pass forward
        found_one = False
        for i in range(len(possible_pass_targets)):
            if (random.uniform(0, 10) < 5):
                self.passing_receiver_position = possible_pass_targets[i]
                found_one = True

        if (not found_one):
            self.passing_receiver_position = possible_pass_targets[0]

        # Take control of them
        self.passing_kicker_position.take_control()
        self.passing_receiver_position.take_control()

    def on_exit_passing(self):
        self.remove_subbehavior('simple_pass')

        self.passing_release_robots()

        # Isn't needed for the specific passing thing, but used anyways
        self.ctr = 0

    # Should be called automatically when play finishes
    def passing_release_robots(self):
        self.passing_kicker_position.release_control()
        self.passing_receiver_position.release_control()

    #####################################
    ####                             ####
    #### Overtake Play State Machine ####
    ####                             ####
    #####################################

    def on_enter_overtake(self):
        # Will be called by parents class
        self.overtake_request_robots()

        self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(self.overtake_receiver_position.target_pos), 'overtake_first_pass', True, 10)

    # Standardized function rn to select which robots we want
    def overtake_request_robots(self):
        # Find place to pass
        # Take position control
        possible_pass_targets = self.position_with_ball.pass_positions

        is_fullback = self.position_with_ball in self.fullbacks
        is_midfielder = self.position_with_ball in self.midfielders

        # Assume is a midfielder if not a fullback
        target_position_list = self.midfielders if is_fullback else self.strikers

        final_pass_target = None

        for pass_target in possible_pass_targets:
            if (pass_target in target_position_list):
                final_pass_target = pass_target
                break

        # Save which two positions we steal
        self.overtake_kicker_position = self.position_with_ball
        self.overtake_receiver_position = final_pass_target

        # Take control of them
        self.overtake_kicker_position.take_control()
        self.overtake_receiver_position.take_control()

    def on_exit_overtake(self):
        self.remove_subbehavior('overtake_first_pass')

    def on_enter_overtake_second_pass(self):
        new_pass_target = self.overtake_receiver_position.target_pos + robocup.Point(-.5, .4)
        
        self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(new_pass_target), 'overtake_second_pass', True, 10)

    def on_exit_overtake_second_pass(self):
        self.remove_subbehavior('overtake_second_pass')

        self.overtake_release_robots()

        self.ctr = 0

    # Should be called automatically when play finishes
    def overtake_release_robots(self):
        self.overtake_kicker_position.release_control()
        self.overtake_receiver_position.release_control()
    
    ###################################
    ####                           ####
    #### Switch Play State Machine ####
    ####                           ####
    ###################################

    def on_enter_switch(self):
        # Will be called by parents class
        self.switch_request_robots()

        self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(self.switch_receiver_position.target_pos), 'switch_pass', True, 10)

    # Standardized function rn to select which robots we want
    def switch_request_robots(self):
        # Get row in front of this one and see if it has two or more robots
        # If so, pass to the furthest one
        # Else, try current row, and pass to furthest one

        is_striker = self.position_with_ball in self.strikers
        is_midfielder = self.position_with_ball in self.midfielders
        is_fullback = self.position_with_ball in self.fullbacks

        rows = [self.strikers, self.midfielders, self.fullbacks]

        row_to_check = 0
        if (is_striker or is_midfielder):
            row_to_check = 0
        else: # is_fullback
            row_to_check = 1

        found_target = False
        target = None

        while (not found_target):
            # Not more than one in that row
            # SKip and find one with more than 1
            if (len(rows[row_to_check]) <= 1):
                row_to_check += 1

            furthest_dist = 0
            furthest_pos = None

            for position in rows[row_to_check]:
                dist = (position.target_pos - self.position_with_ball.target_pos).mag()

                if (dist > furthest_dist):
                    furthest_dist = dist
                    furthest_pos = position
                    found_target = True

            target = furthest_pos

        # Save which two positions we steal
        self.switch_kicker_position = self.position_with_ball
        self.switch_receiver_position = target

        # Take control of them
        self.switch_kicker_position.take_control()
        self.switch_receiver_position.take_control()

    def on_exit_switch(self):
        self.remove_subbehavior('switch_pass')

        self.switch_release_robots()

        self.ctr = 0

    # Should be called automatically when play finishes
    def switch_release_robots(self):
        self.switch_kicker_position.release_control()
        self.switch_receiver_position.release_control()

    ##################################
    ####                          ####
    #### Cross Play State Machine ####
    ####                          ####
    ##################################

    def on_enter_cross(self):
        # Will be called by parents class
        self.cross_request_robots()

        # Get corner kick location
        corner_cross_pos = robocup.Point(self.width/2 * .9, self.length * .9)
        if (main.ball().pos.x < 0):
            corner_cross_pos.x = -corner_cross_pos.x

        support_pos = robocup.Point(0, self.length * .7)
        
        back_pass_pos = corner_cross_pos*.8

        future_one_touch_pos = robocup.Point(0, self.length * .85)

        self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(corner_cross_pos), 'cross_setup', True, 10)
        self.add_subbehavior(skills.move.Move(support_pos), 'support_move', True, 10)
        self.add_subbehavior(skills.move.Move(back_pass_pos), 'back_pass_move', True, 10)

    # Standardized function rn to select which robots we want
    def cross_request_robots(self):
        # Go from current position into one of the corners
        # Take current robot with ball
        # And striker on that side (or midfielder)

        is_left_side = main.ball().pos.x < 0

        is_striker = self.position_with_ball in self.strikers

        # Assume is a midfielder if not a fullback
        target_position_list = self.strikers if not is_striker else self.midfielders

        # Closest person is either the first or last in list
        closest_position_idx = 0 if is_left_side else len(target_position_list) - 1
        closest_position = target_position_list[closest_position_idx]

        # Since all robots list is from left to right front to back
        # we can just loop through that until we find a robot not being used in offense
        # Should be closest one in front to help setup
        extra_support_position = None
        for position in self.all_positions:
            if (position is not self.position_with_ball and
                position is not closest_position):
                extra_support_position = position
                break

        # Back pass should be closet fullback
        # Can reverse list if on right side
        # Warning: This makes assumptions about formation
        fullbacks = self.fullbacks if is_left_side else reversed(self.fullbacks)
        back_pass_position = None
        for fullback in fullbacks:
            back_pass_position = fullback
            break

        print(extra_support_position is closest_position)

        self.cross_shooter_position = self.position_with_ball
        self.cross_crosser_position = closest_position
        self.cross_extra_support_position = extra_support_position
        self.cross_back_pass_position = back_pass_position

        # Take control of them
        self.cross_shooter_position.take_control()
        self.cross_crosser_position.take_control()
        self.cross_extra_support_position.take_control()
        self.cross_back_pass_position.take_control()

    def on_exit_cross(self):
        self.remove_subbehavior('cross_setup')

    def on_enter_cross_into_middle(self):
        # Can do some extra movement here as well if we want
        self.add_subbehavior(tactics.one_touch_pass.OneTouchPass(), 'one_touch', True, 10)

    def on_exit_cross_into_middle(self):
        self.remove_subbehavior('one_touch')
        self.remove_subbehavior('support_move')
        self.remove_subbehavior('back_pass_move')

        self.cross_release_robots()

        self.ctr = 0

    # Should be called automatically when play finishes
    def cross_release_robots(self):
        self.cross_shooter_position.release_control()
        self.cross_crosser_position.release_control()
        self.cross_extra_support_position.release_control()
        self.cross_back_pass_position.release_control()

    #################################
    ####                         ####
    #### Pass Play State Machine ####
    ####                         ####
    #################################

    def on_enter_shoot(self):
        # Will be called by parents class
        self.shoot_request_robots()

        kick = skills.pivot_kick.PivotKick()
        kick.target = constants.Field.TheirGoalSegment.center()

        self.add_subbehavior(kick, 'shooter', True, 10)

    # Standardized function rn to select which robots we want
    def shoot_request_robots(self):
        self.shoot_kicker_position = self.position_with_ball
        self.shoot_kicker_position.take_control()

    def on_exit_shoot(self):
        self.remove_subbehavior('shooter')

        self.shoot_release_robots()

        self.ctr = 0

    # Should be called automatically when play finishes
    def shoot_release_robots(self):
        self.shoot_kicker_position.release_control()

    @classmethod
    def score(cls):
        if (not main.game_state().is_playing()):
            return float("inf")
        return 10