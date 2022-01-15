import stp
from rj_gameplay.skill import receive, line_kick, pivot_kick

class PasserRole(stp.role.Role):
    def __init__(self, robot: stp.rc.Robot) -> None:
        super().__init__(robot)

        self.receive_skill = None

        # TODO: make FSM class (or at least use enum instead of str literals)
        self._state = "init"

    @property
    def pass_ready(self):
        return self._state == "pass_ready"

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
            self._state = "receiving"
        elif self._state == "receiving":
            intent = self.receive_skill.tick(world_state)
            if self.receive_skill.is_done(world_state):
                self._state = "pass_ready"
        elif self._state == "pass_ready":
            if #<signal-from-tactic-that-recv-is-ready>:
                # dribble until the receiver is ready
                intent = #<piv-kick>

            if #<kick-done>:
                self._state = "kick_done"
        elif self._state == "kick_done":
            pass

        return intent

    def is_done(self, world_state) -> bool:
        return self._state == "kick_done"
