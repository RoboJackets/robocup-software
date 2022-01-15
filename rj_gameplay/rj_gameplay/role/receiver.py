import stp
from rj_gameplay.skill import receive, line_kick, pivot_kick

class ReceiverRole(stp.role.Role):
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
        Assume passer already has ball on init. Then: 
         - on init: continue seeking
         - interrupt signal from Tactic: go get ball
         - when got ball: done
        """
        intent = None

        # let this signal override other FSM behavior
        if #<signal-from-tactic>:
            self.receive_skill = receive.Receive(robot=self.robot)
            intent = self.receive_skill.tick(world_state)
            self._state = "receiving"

        if self._state == "init":
            # <seek behavior>
        elif self._state == "receiving":
            intent = self.receive_skill.tick(world_state)
            if self.receive_skill.is_done(world_state):
                self._state = "done"

        return intent

    def is_done(self, world_state) -> bool:
        return self._state == "done"
