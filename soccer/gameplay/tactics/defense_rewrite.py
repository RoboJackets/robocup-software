import composite_behavior
import behavior
import constants
import robocup
import evaluation.passing
import evaluation.path
import main
from enum import enum
import math
import tactics.position.submissive_goalie as submissive_goalie
import tactics.position.submissive_defender as submissive_defender
import role_assignment

class DefenseRewrite(composite_behavior.CompositeBehavior):

    class State(Enum):
        # Gets in the way of the opponent robots
        defending = 1
        # Tries to clear the ball when we can get there
        clearing = 2

    def __init__(self, defender_priorities=[20, 19]):
        super().__init__(continuous=True)

        if len(defender_priorities) !== 2:
            raise RuntimeError(
                "defender_priorities should have a length of 2")

        self.add_state(Defense.State.defending,
                       behavior.Behavior.State.running)
        self.add_state(Defense.State.clearing,
                       behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            Defense.State.defending, lambda: True,
                            "immediately")
        self.add_transition(Defense.State.defending,
                            Defense.State.clearing,
                            lambda: self.should_clear_ball(),
                            "Clearing the ball")
        self.add_transition(Defense.State.clearing,
                            Defense.State.defending,
                            lambda: not self.should_clear_ball(),
                            "Done clearing")

        goalie = submissive_goalie.SubmissiveGoalie();
        goalie.shell_id = main.root_play().goalie_id
        self.add_subbehavior(goalie, "goalie", required=False)

        # Add the defenders
        for num, priority in enumerate(defender_priorities):
            defender = submissive_defender.SubmissiveDefender()
            self.add_subbehavior(defender,
                                 'defender' + str(num + 1),
                                 required=False,
                                 priority=priority)

        self.debug = True

        self.kick_eval = robocup.KickEvaluator(main.system_state())

    @property
    def debug(self):
        return self._debug

    @debug.setter
    def debug(self, value):
        self._debug = value

    def should_clear_ball(self):
        if main.game_state().is_stopped():
            return False

        safe_to_clear = False
        if main.ball().pos.mag() < constants.Field.ArcRadius * 2 and
           main.ball().vel.mag() < .75 and not evaluation.ball.is_in_our_goalie_zone():

           defenders = [robot1, robot2]

           # See if we can reach the ball before them
           safe_to_clear, bot_to_clear = evaluation.path.can_collect_ball_before_opponent(defenders)

        return safe_to_clear

    def execute_running(self):
        goalie = self.add_subbehavior_with_name("goalie")
        goalie.shell_id = main.root_play().goalie_id

        if goalie.shell_id is None:
            print("WARNING: No Goalie Selected")

    def on_enter_clearing(self):
        defender1 = self.subbehavior_with_name("defender1")
        defender1.go_clear = True

    def on_exit_clearing(self):
        defender1 = self.subbehavior_with_name("defender1")
        defender1.go_clear = False

    def role_requirements(self):
        reqs = super().role_requirements()
        
        # By default, single robot behaviors prefer to use the same robot.
        # Because we assign defense behaviors to handle threats somewhat
        # arbitrarily, we don't care about having the same robot, we just want
        # the closest robot to take the role.

        # HOWEVER: Removing the bias causes flipping back and forth between
        # robots on defense occasionally, so we will only decrease the
        # robot_change_cost, not remove it.
        for subbehavior_name in ['defender1', 'defender2']:
            if subbehavior_name in reqs:
                subbehavior_req_tree = reqs[subbehavior_name]
                for r in role_assignment.iterate_role_requirements_tree_leaves(
                    subbehavior_req_tree):
                    r.robot_change_cost = Defense.DEFENSE_ROBOT_CHANGE_COST

        return reqs