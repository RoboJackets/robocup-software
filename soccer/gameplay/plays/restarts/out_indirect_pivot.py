import standard_play
import behavior
import skills
import tactics
import robocup
import constants
import main
import math
from enum import Enum
import evaluation
from evaluation.passing import eval_pass, eval_chip 
import evaluation.space
import evaluation.situation
import time


class OurIndirectPivot(standard_play.StandardPlay):

    LAST_START=None
    MAX_RUNTIME = 15
    class State(Enum):
        preparing=0
        passing=1
        kicking=2

    MinChipRange = 0.3
    MaxChipRange = 3.0
    ChipperPower = 1
    TargetSegmentWidth = 1.5
    MaxKickSpeed = 1
    MaxKickAccel = 1
    Running = False



    def __init__(self, indirect=None):
        super().__init__(continuous=True)
        OurIndirectPivot.LAST_START = int(time.time())
        self.ball_start_pos = main.ball().pos
        self.ball_has_left = False
        self.receive_point = self.pick_pass_spot()
        self.ball_near_receive_point_time = None

        # setup a line kick skill to replace the pivotkick since a pivot would easily cause a double touch
        self.shot_kicker = skills.pivot_kick.PivotKick()
        self.shot_kicker.chip_power = OurIndirectPivot.ChipperPower  # TODO: base this on the target dist from the bot
        self.shot_kicker.min_chip_range = OurIndirectPivot.MinChipRange
        self.shot_kicker.max_chip_range = OurIndirectPivot.MaxChipRange
        self.shot_kicker.max_speed = OurIndirectPivot.MaxKickSpeed
        self.shot_kicker.max_accel = OurIndirectPivot.MaxKickAccel

        self.kicker = skills.line_kick.LineKick()
        self.kicker.chip_power = OurIndirectPivot.ChipperPower  # TODO: base this on the target dist from the bot
        self.kicker.min_chip_range = OurIndirectPivot.MinChipRange
        self.kicker.max_chip_range = OurIndirectPivot.MaxChipRange
        self.kicker.max_speed = OurIndirectPivot.MaxKickSpeed
        self.kicker.max_accel = OurIndirectPivot.MaxKickAccel

        # larger avoid ball radius for line kick setup so we don't run over the ball backwards
        self.kicker.setup_ball_avoid = constants.Field.CenterRadius - constants.Robot.Radius
        self.kicker.drive_around_dist = constants.Field.CenterRadius - constants.Robot.Radius

        # create a one touch pass behavior with line kick as the skill
        self.pass_bhvr = tactics.coordinated_pass.CoordinatedPass(skillkicker=(self.kicker, lambda x: True))

        self.add_state(OurIndirectPivot.State.passing,
                       behavior.Behavior.State.running)
        self.add_state(OurIndirectPivot.State.kicking, behavior.Behavior.State.running)
        self.add_state(OurIndirectPivot.State.preparing, behavior.Behavior.State.running)

        # add transistions for when the play is done
        self.add_transition(behavior.Behavior.State.start,
                            OurIndirectPivot.State.passing, lambda: True,
                            'immediately')

        #self.add_transition(OurIndirectPivot.State.preparing,
        #                    OurIndirectPivot.State.passing, lambda: self.subbehavior_with_name('setupKick').is_done_running(),
        #                    'move_completed')

        self.add_transition(OurIndirectPivot.State.passing,
                            OurIndirectPivot.State.kicking,
                            lambda: self.pass_bhvr.is_done_running() or self.ball_has_left, 'Pass Completed')

        self.add_transition(OurIndirectPivot.State.kicking,
                            behavior.Behavior.State.completed,
                            lambda: False, 'Shot Completed')

    @classmethod
    def score(cls):
        gs = main.game_state()
        if not (gs.is_playing() or gs.is_ready_state()) or (OurIndirectPivot.LAST_START is not None and (time.time() - OurIndirectPivot.LAST_START) > OurIndirectPivot.MAX_RUNTIME):
            OurIndirectPivot.Running = False
            OurIndirectPivot.LAST_START = None
            return float('inf')
        # enter play when doing a corner kick or stay in it even if we manipulate the ball
        if not OurIndirectPivot.Running and OurIndirectPivot.LAST_START is None and (gs.is_ready_state() and gs.is_our_free_kick() and main.ball().pos.y < (
                constants.Field.Length - 1.2) and main.ball().pos.y >= constants.Field.Length/2 and (gs.is_our_direct() and main.ball().pos.y >= constants.Field.Length-3 and abs(main.ball().pos.x)) < 1 ):
            OurIndirectPivot.Running = True
            return 0
        elif OurIndirectPivot.Running or (gs.is_ready_state() and gs.is_our_free_kick() and main.ball().pos.y < (
                constants.Field.Length - 1.2) and main.ball().pos.y >= constants.Field.Length/2 ):
            OurIndirectPivot.Running = True
            return 0
        else:
            return float("inf")

    @classmethod
    def is_restart(cls):
        return True

    def on_enter_preparing(self):
        self.add_subbehavior(skills.move.Move((main.ball().pos + (main.ball().pos -robocup.Point(0,constants.Field.Length)).normalized()/2)), 'setupKick', priority=25)
        self.add_subbehavior(skills.move.Move(self.receive_point), 'setupReceive', priority=19)
        backup_point = self.receive_point + (main.ball().pos - self.receive_point).normalized()
        self.add_subbehavior(skills.move.Move(backup_point), 'Backup', priority=1)

    def on_exit_preparing(self):
        self.remove_all_subbehaviors()

    def on_enter_passing(self):
        # start the actual pass
        self.add_subbehavior(self.pass_bhvr, 'pass', priority=20)
        self.receive_point = self.pick_pass_spot()
        backup_point = main.ball().pos - (main.ball().pos - self.receive_point)-(main.ball().pos - self.receive_point).normalized()
        backup_point.rotate(main.ball().pos,-math.pi/32)
        #backup_point = main.ball().pos + backup_point
        #backup_point = backup_point.normalized()
        print(evaluation.situation.goal_per_minute_required())
        backup_req = True if evaluation.situation.goal_per_minute_required()>=1 else False
        self.add_subbehavior(skills.move.Move(backup_point), 'Backup', priority=1, required=backup_req)
        self.pass_bhvr.receive_point = self.receive_point
        self.pass_bhvr.use_chipper = True #self.evaluate_chip(self.receive_point)
        

    def execute_passing(self):
        if (main.ball().pos - self.ball_start_pos).mag() > 3:
            self.ball_has_left = True 
        #if self.pass_bhvr.state == tactics.coordinated_pass.CoordinatedPass.State.preparing:
            #self.pass_bhvr.receive_point = self.pick_pass_spot()
            #self.pass_bhvr.use_chipper = self.evaluate_chip(self.pass_bhvr.receive_point)

    def ball_near_receive_point(self):
        return (self.receive_point - main.ball().pos).mag() <= .75

    # Wasn't working- taken out
    # Preferable to seeing if ball has moved a lot if it can get working
    def ball_close_enough_to_receive(self):
        if self.ball_near_receive_point():
            if self.ball_near_receive_point_time is None:
                self.ball_near_receive_point_time = time.time()
                return False
            else:
                if time.time() - self.ball_near_receive_point_time >= 1:
                    return True
                else:
                    return False 
        else:
            self.ball_near_receive_point_time = None
            return False


    def on_enter_kicking(self):
        self.remove_subbehavior('pass')
        self.add_subbehavior(self.shot_kicker, 'Shot')

        if abs(main.ball().pos.x)>2:
            rebound_point = robocup.Point(-2*main.ball().pos.x/main.ball().pos.x,main.ball().pos.y)
        elif abs(main.ball().pos.x)<.25:
            rebound_point = robocup.Point(-(.5+main.ball().pos.x),main.ball().pos.y)
        else:
            rebound_point = robocup.Point(-main.ball().pos.x,main.ball().pos.y)
        print("Rebounding at : ", rebound_point)
        self.add_subbehavior(skills.move.Move(rebound_point), 'Rebound', required=False, priority=5)

    def execute_kicking(self):
        if self.subbehavior_with_name('Shot').state in [behavior.Behavior.State.completed,behavior.Behavior.State.failed]:
            print('Failed Shot on PivotKick Indirect')
            OurIndirectPivot.Running = False

    def on_exit_kicking(self):
        self.remove_all_subbehaviors()

    def pick_pass_spot(self):
        tmp = evaluation.space.get_best_downfield_space_point(min_radius=constants.OurChipping.MIN_CAPTURE_DISTANCE, 
                                    max_radius=constants.OurChipping.MAX_CAPTURE_DISTANCE, 
                                    radius_resolution=.25, 
                                    min_upfield_distance=.5, 
                                    min_downfield_distance=.25)
        if tmp is None:
            print("No good evaluations found, using a static point")
            return robocup.Point(math.copysign(1,main.ball().pos.x),constants.Field.Length-1.6)
        else:
            return tmp
        

    def evaluate_chip(self, receive_point):
        bp = main.ball().pos
        ex_robots = []
        kick_p = eval_pass(bp, receive_point, excluded_robots=ex_robots) 
        #print("Kick probability is {}".format(kick_p))
        if kick_p < .6:
            chip_p = eval_chip(bp, receive_point, excluded_robots=ex_robots)
            #print("Chip probability is {}".format(chip_p))
            if chip_p > kick_p+.05:
                return True
        return False

    @classmethod
    def is_restart(cls):
        return True