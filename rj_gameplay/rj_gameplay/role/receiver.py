import stp.role
import stp.rc

from rj_gameplay.skill import receive

from rj_msgs.msg import RobotIntent


class ReceiverRole(stp.role.Role):
    def __init__(self, robot: stp.rc.Robot) -> None:
        super().__init__(robot)

        self.receive_skill = None

        # TODO: make FSM class (or at least use enum instead of str literals)
        self._state = "init"

        self._target_point = None

    @property
    def pass_ready(self):
        return self._state == "pass_ready"

    def set_receive_pass(self):
        self._state = "receive_pass"
        self.receive_skill = receive.Receive(robot=self.robot)

    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:
        """
        Assume passer already has ball on init. Then:
         - on init: continue seeking
         - interrupt signal from Tactic: go get ball
         - when got ball: done
        """

        intent = None

        if self._state == "init":
            # do seek behavior
            pass
        elif self._state == "receive_pass":
            intent = self.receive_skill.tick(world_state)
            if self.receive_skill.is_done(world_state):
                self._state = "done"
                # end FSM

        return intent

    def is_done(self, world_state) -> bool:
        return self._state == "done"
