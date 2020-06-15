import unittest
import fsm
import enum


class MyFsm(fsm.StateMachine):
    class State(enum.Enum):
        start = 1
        running = 2
        done = 3

    class SubState(enum.Enum):
        running_substate = 1

    def __init__(self):
        super().__init__(start_state=MyFsm.State.start)
        self.add_state(MyFsm.State.start)
        self.add_state(MyFsm.State.running)
        self.add_state(MyFsm.State.done)
        self.add_state(MyFsm.SubState.running_substate, MyFsm.State.running)

        self.add_transition(MyFsm.State.start, MyFsm.SubState.running_substate,
                            lambda: True, 'immediately')
        self.add_transition(MyFsm.SubState.running_substate, MyFsm.State.done,
                            lambda: True, 'immediately')

        self._log = []

    # log all fsm method calls
    def on_enter_start(self):
        self._log.append("on_enter_start")

    def on_enter_running(self):
        self._log.append("on_enter_running")

    def on_enter_done(self):
        self._log.append("on_enter_done")

    def on_enter_running_substate(self):
        self._log.append("on_enter_running_substate")

    def on_exit_start(self):
        self._log.append("on_exit_start")

    def on_exit_running(self):
        self._log.append("on_exit_running")

    def on_exit_done(self):
        self._log.append("on_exit_done")

    def on_exit_running_substate(self):
        self._log.append("on_exit_running_substate")

    def execute_start(self):
        self._log.append("execute_start")

    def execute_running(self):
        self._log.append("execute_running")

    def execute_done(self):
        self._log.append("execute_done")

    def execute_running_substate(self):
        self._log.append("execute_running_substate")


class TestFsm(unittest.TestCase):
    def test_transisions(self):
        # """When transitioning to or from a state, the fsm should
        # call the parent transition methods as well as the child ones"""

        fsm = MyFsm()
        while fsm.state != MyFsm.State.done:
            fsm.spin()

        expected_log = [
            "on_enter_start", "execute_start", "on_exit_start",
            "on_enter_running", "on_enter_running_substate", "execute_running",
            "execute_running_substate", "on_exit_running",
            "on_exit_running_substate", "on_enter_done", "execute_done"
        ]

        self.assertEqual(expected_log, fsm._log)

    def test_ancestor_chain(self):
        """see if the ancestors_of_state() method works"""

        fsm = MyFsm()
        self.assertEqual(fsm.ancestors_of_state(MyFsm.State.done), [])
        self.assertEqual(
            fsm.ancestors_of_state(MyFsm.SubState.running_substate),
            [MyFsm.State.running])
