import main
import robocup
import behavior
import constants
import enum

import standard_play

import evaluation.passing_positioning

import tactics.coordinated_pass
import tactics.defensive_forward

import skills.moving_pass_receive
import skills.move
import skills.capture

# Offense:

# Collect the ball / Defense
#         2 Defending goal (1 Goalie)
#         3 Left
#         Defend another shot from additional strikers
#         (User distance to goal / angle to figure out the chance)
#         If < Cutoff, move said defending robot to block pass
#         Left over ones defend pass

#         Once ball is collected move to Dribble

# Dribble
#         Move all offensive robots up the field
#         Test recieve positions and find best one
#         Dribble if nothing good is found
#             Find better definitions on when to dribble

#         Calculate pass
#             Chance to pass
#                 Basic chance of intersection (Pass distance accounted for indirectly)
#             pos_heuristic
#                 Weight center (Linear coeff)
#                 Weight space  (Triweight coff)
#                 Weight proximity to ball location (N-Power)
#                 Weight angle between goal post (linear coeff) (Angle between is already nonlinear)
#                 Weight shot super high if one is availible (touchpassPositioning)
#                 Change weight distribution based on their defense
#                     Evaluate % time spent doing man on man vs zone (Add other defenses as created)
#                     Man on man -> Space is higher
#                     Zone       -> Zone is higher
#                 Allow for pass back due to cornering
#                     Back pass must be open  significantly
#                     May be accounted for with space / center heuristic already
#             Add some way to get longer pass being solved first when in the same direction

#         Calculate shot (Use build-in for now)
#             Basic motion prediction
#                 Predict velocity (Then acceleration)
#             Ball prediction
#                 Predict along path to figure out closeness of interception
#             Distance
#                 Distance from goal (Indirectly weighted through other means, but allows for a quick cutoff if needed)
#             Angle
#                 Linearly decrease chance besed on normal distribution (Could be more advanced if needed)

#         Calculate clear
#             Dist_to_our_goal
#                 Linear weight
#             Chance to pass
#                 Basic chance of intersection
#             pos_heuristic
#                 Maybe, sill haven't decided
#                 Most likely like the one above (Maybe try inverse)


# Passing
#     We have chosen to pass
#     Find the best robot to approach ball
#         Look into weighting based on angle off of ball
#         We want ball to be approached from opposite of goal
#     Prepare other robots to move out of the way
#         Drawing out the defense
#         (Depends on the type of defense)

# Shooting
#     We have chosen to shoot
#     Aim at best location and shoot
#         Basic motion prediction
#         Ball prediction
#         Angle
#         Largest section adviable

# Clearing
#     We have chosen to clear
#     Aim of robot CHIP distance away
#     Find most open robot
#     if < cutoff & no open robots
#     Just find most open area

# These three may be tied into the basic passing logic and all that will change is the receive / collect object
# PassinMotion
#     Pass has been kicked (Until just about to be collected)
#     Move robot into interception
#     Have a robots move to block interception
#         Direction of motion of intercepting robot
#     Move other robots into Defensive->Offensive positions
#         Based on the %pass completed

# PassCollecting
#     Pass is about to be collected
#     Continue collection and predict whether to one hit touch or settle
#         Based on openness of the player and path to goal
#     Shot open?
#         Take the one touch (evalPass)
#     Teammate open with perfect shot?
#         One touch pass (touchpassPositioning)
#     Neither at a very good percentage
#         Settle with dribble

# Onetouch
#     if shot
#         Shoot
#     if pass
#         Pass



# TODO: Fix everything to follow standards

class AdaptiveFormation(standard_play.StandardPlay):
    class State(enum.Enum):
        # Collect the ball / Full court defense
        collecting = 1
        # Dribble for a second and prepare to pass / shoot / clear
        dribbling = 2
        # Pass when someone is open
        passing = 3
        # Shoot when chances are high
        shooting = 4
        # Clear when pass / dribble is worse and we are in our own zone
        clearing = 5

        # Pass is in motion, move to collect pass, other robots move into position
        passInMotion = 6
        # Check if one touch goal is better than settling
        passCollecting = 7
        # One touch shot
        oneTouch = 8

    def __init__(self):
        super().__init__(continuous=False)

        for s in AdaptiveFormation.State:
            self.add_state(s, behavior.Behavior.State.running)

        # Min score to pass
        self.dribble_to_pass_cutoff = 0.1
        # Min score to shoot
        self.dribble_to_shoot_cutoff = 0.1
        # Dribbling skill
        self.dribbler = None
        # Dribble start point
        self.dribble_start_pt = robocup.Point(0,0)
        # Max dribble distance per the rules with 10% wiggle room
        self.max_dribble_dist = 1 * .9

        # Min field Y to clear
        self.clear_field_cutoff = constants.Field.Length / 5 # in our 20%
        self.clear_distance_cutoff = constants.Field.Length / 10 ## if closest robot is 10% of the field away, don't clear


        # The minimum increase from one cycle to the next to hold off Passing/Shooting/Clearing
        self.IncreasingChancesCutoff = 0.05
        # State Deciding Variables
        self.shot_chance = 0
        self.pass_score = 0
        # Prev State Deciding Variables
        self.prev_shot_chance = 0
        self.prev_pass_score = 0
        self.check_dribbling_timer = 0
        self.check_dribbling_timer_cutoff = 100


        # field_pos_weights: (Centerness, Distance to their goal, Angle off their goal)
        # [type]_weights: (space, field_position, shot_chance, kick_proximity)

        # Weights for the field positioning
        self.field_pos_weights = (0.01, 3, 0.02)
        # Weights for finding best pass
        self.passing_weights = (2, 2, 15, 1)
        # Weights for finding where to dribble to
        self.dribbling_weights = (4, 2, 15, 1)
        # Weights to find where to chip the ball
        self.chip_field_pos_weights = (0.1, .2, 0.02)
        self.chip_pass_weights = (2, 10, 0, 10)

        # Min onetouch score
        self.one_touch_shot_cutoff = 0.9

        # Min distance to switch phases
        self.pass_collecting_dist = 0.5

        # State Transition Variable
        self.pass_target = None


        # Add transitions
        self.add_transition(behavior.Behavior.State.start,
                            AdaptiveFormation.State.collecting,
                            lambda: True,
                            'immediately')

        self.add_transition(AdaptiveFormation.State.collecting,
                            AdaptiveFormation.State.dribbling,
                            lambda: False,#self.subbehavior_with_name('capture').is_done_running(),
                            'Ball Collected')

        self.add_transition(AdaptiveFormation.State.dribbling,
                            AdaptiveFormation.State.passing,
                            lambda: self.should_pass_from_dribble(),
                            'Passing')

        self.add_transition(AdaptiveFormation.State.dribbling,
                            AdaptiveFormation.State.shooting,
                            lambda: self.should_shoot_from_dribble(),
                            'Shooting')

        self.add_transition(AdaptiveFormation.State.dribbling,
                            AdaptiveFormation.State.clearing,
                            lambda: self.should_clear_from_dribble(),
                            'Clearing')

        # Passing states
        self.add_transition(AdaptiveFormation.State.passing,
                            AdaptiveFormation.State.passInMotion,
                            lambda: self.subbehavior_with_name('pass').state == behavior.Behavior.State.completed,
                            'Pass Kicked')

        self.add_transition(AdaptiveFormation.State.passInMotion,
                            AdaptiveFormation.State.passCollecting,
                            lambda: True,
                            'Pass About to be Collected')

        self.add_transition(AdaptiveFormation.State.passCollecting,
                            AdaptiveFormation.State.oneTouch,
                            lambda: False,
                            'One Touch Shot')

        self.add_transition(AdaptiveFormation.State.passCollecting,
                            AdaptiveFormation.State.dribbling,
                            lambda: True,
                            'Pass Settled')

        # Reset to collecting when ball is lost at any stage
        self.add_transition(AdaptiveFormation.State.dribbling,
                            AdaptiveFormation.State.collecting,
                            lambda: False,
                            'Dribble: Ball Lost')
        self.add_transition(AdaptiveFormation.State.passing,
                            AdaptiveFormation.State.collecting,
                            lambda: self.subbehavior_with_name('pass').state == behavior.Behavior.State.failed,
                            'Passing: Ball Lost')
        self.add_transition(AdaptiveFormation.State.shooting,
                            AdaptiveFormation.State.collecting,
                            lambda: self.subbehavior_with_name('kick').state == behavior.Behavior.State.completed or \
                                    self.subbehavior_with_name('kick').state == behavior.Behavior.State.failed,
                            'Shooting: Ball Lost / Shot')
        self.add_transition(AdaptiveFormation.State.clearing,
                            AdaptiveFormation.State.collecting,
                            lambda: False,
                            'Clearing: Ball Lost')
        self.add_transition(AdaptiveFormation.State.passInMotion,
                            AdaptiveFormation.State.collecting,
                            lambda: False,
                            'PassInMotion: Ball Lost')
        self.add_transition(AdaptiveFormation.State.passCollecting,
                            AdaptiveFormation.State.collecting,
                            lambda: False,
                            'PassCollecting: Ball Lost')
        self.add_transition(AdaptiveFormation.State.oneTouch,
                            AdaptiveFormation.State.collecting,
                            lambda: False,
                            'OneTouch: Ball Lost / Shot')


    def should_pass_from_dribble(self):

        # If pass is above cutoff and we dont have a good shot
        if (self.pass_score > self.dribble_to_pass_cutoff and \
            self.shot_chance < self.dribble_to_shoot_cutoff):
            print("Pass : " + str(self.pass_score) + " Shot : " + str(self.shot_chance))
            return True

        # Force pass if we are near our max dribble dist
        dribble_dist = (main.ball().pos - self.dribble_start_pt).mag()
        if (dribble_dist > self.max_dribble_dist):
            return True

        # Under cutoff
        return False

    def should_shoot_from_dribble(self):

        # If shot chance is improving significantly, hold off a second
        if (self.prev_shot_chance + self.IncreasingChancesCutoff < self.shot_chance):
            return False

        # If shot is above cutoff
        if (self.shot_chance > self.dribble_to_shoot_cutoff):
            print("Pass : " + str(self.pass_score) + " Shot : " + str(self.shot_chance))
            return True

        # Decreasing and under cutoff
        return False


    def should_clear_from_dribble(self):
        # If outside clear zone
        if (self.dribbler.pos.y > self.clear_field_cutoff):
            return False

        # If pass chances are getting better, hold off
        if (self.prev_pass_score + IncreasingChancesCutoff < self.pass_score):
            return False

        # TODO: See if there is space to dribble
        closest_distance = (evaluation.opponent.get_closest_opponent(main.ball().pos, 1).pos - main.ball().pos).mag()
        if (closest_distance > self.clear_distance_cutoff):
            return False

        return True

    def on_enter_collecting(self):
        defensive_forward = tactics.defensive_forward.DefensiveForward()
        self.add_subbehavior(defensive_forward, 'defend', required=True)

        # TODO: Setup offense to be defenders
        # One defense that zone point
        # Two mark the robots
        # Markers block the robots

        # For testing purposes only:
        #capture = skills.capture.Capture()
        #self.add_subbehavior(capture, 'capture', required=True)

    def on_exit_collecting(self):
        self.remove_all_subbehaviors()

    def on_enter_dribbling(self):
        self.dribbler = skills.dribble.Dribble()
        self.dribble_start_pt = main.ball().pos

        # Dribbles toward the best receive point
        self.dribbler.pos, _ = evaluation.passing_positioning.eval_best_receive_point(
                                                    main.ball().pos, None, main.our_robots(),
                                                    self.field_pos_weights, self.dribbling_weights, False)
        self.add_subbehavior(self.dribbler, 'dribble', required=True)

        self.check_dribbling_timer = 0

    def execute_dribbling(self):
        # Find the closest bot weighting the ones in front of the ball more
        closest_bot = evaluation.opponent.get_closest_opponent(main.ball().pos, 0.9)

        # Grab best pass
        self.pass_target, self.pass_score = evaluation.passing_positioning.eval_best_receive_point(
                                                    main.ball().pos, None, main.our_robots(),
                                                    self.field_pos_weights, self.passing_weights, False)

        # Grab shot chance
        self.shot_chance = evaluation.shooting.eval_shot(main.ball().pos)

        # Recalculate dribbler pos
        self.check_dribbling_timer += 1
        if (self.check_dribbling_timer > self.check_dribbling_timer_cutoff):
            self.check_dribbling_timer = 0
            self.dribbler.pos, _ = evaluation.passing_positioning.eval_best_receive_point(
                                                    main.ball().pos, None, main.our_robots(),
                                                    self.field_pos_weights, self.dribbling_weights, False)

        # TODO: Get list of top X pass positions and have robots in good positions to reach them
        # Good positions can be definied by offensive / defensive costs
        # Offensive positions move onto the ball in the direction of the goal
        # Defensive cover the center of the field

        # Setup previous values (Basic complementary filter)
        c = .8
        self.prev_shot_chance = c*self.shot_chance + (1-c)*self.prev_shot_chance
        self.prev_pass_score = c*self.pass_score + (1-c)*self.prev_pass_score


    def on_exit_dribbling(self):
        self.remove_all_subbehaviors()

    def on_enter_shooting(self):
        # TODO: Use moving kick when completed
        kick = skills.pivot_kick.PivotKick()
        kick.target = constants.Field.TheirGoalSegment
        kick.aim_params['desperate_timeout'] = 3
        self.add_subbehavior(kick, 'kick', required=False)

    def on_exit_shooting(self):
        self.remove_all_subbehaviors()

    def on_enter_clearing(self):
        # Line kick with chip
        # Choose most open area / Best pass, weight forward
        # Decrease weight on sides of field due to complexity of settling
        pass

    def execute_clearing(self):
        pass

    def on_exit_passing(self):
        self.remove_all_subbehaviors()

    def on_enter_passing(self):
        self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(self.pass_target, skills.moving_pass_receive.MovingPassReceive()), 'pass')

    def on_exit_passing(self):
        self.remove_all_subbehaviors()

    # Move other robots out into position
    # Check to see what type of reception we should do
    # Begin moving robots who are out of position
    # Update in real time
    def on_enter_passInMotion(self):
        pass

    def execute_passInMotion(self):
        pass

    def on_exit_passInMotion(self):
        self.remove_all_subbehaviors()

    def on_enter_passCollecting(self):
        pass

    def on_enter_oneTouch(self):
        pass
