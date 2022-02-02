import stp.role
import stp.rc

from rj_gameplay.skill import receive, pivot_kick  # , line_kick

from rj_msgs.msg import RobotIntent


class PasserRole(stp.role.Role):
    def __init__(self, robot: stp.rc.Robot) -> None:
        super().__init__(robot)

        self.receive_skill = None
        self.pivot_kick_skill = None

        # TODO: make FSM class (or at least use enum instead of str literals)
        self._state = "init"

        self._target_point = None

    @property
    def pass_ready(self):
        return self._state == "pass_ready"

    def set_execute_pass(self, target_point):
        self._state = "init_execute_pass"
        self._target_point = target_point

    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:
        """
        Assume robot does not have ball on init. Then:
         - on init: get ball
         - when got ball: mark pass ready for Tactic, dribble, wait
         - on pass signal from Tactic: pivot_kick to point, let receiver get ball, done
        """

        intent = None
        if self._state == "init":
            self.receive_skill = receive.Receive(robot=self.robot)
            intent = self.receive_skill.tick(world_state)
            self._state = "capturing"
        elif self._state == "capturing":
            intent = self.receive_skill.tick(world_state)
            if self.receive_skill.is_done(world_state):
                self._state = "pass_ready"
        elif self._state == "pass_ready":
            # TODO: dribble until the receiver is ready
            pass
        # this state transition is done by the PassTactic, which is not canonical FSM
        elif self._state == "init_execute_pass":
            # TODO: make these params configurable
            self.pivot_kick_skill = pivot_kick.PivotKick(
                robot=self.robot,
                target_point=self._target_point,
                chip=False,
                kick_speed=4.0,  # TODO: adjust based on dist from target_point
            )
            self._state = "execute_pass"
        elif self._state == "execute_pass":
            intent = self.pivot_kick_skill.tick(world_state)

            if self.pivot_kick_skill.is_done(world_state):
                self._state = "kick_done"
                # end FSM

        return intent

    def is_done(self, world_state) -> bool:
        return self._state == "kick_done"
