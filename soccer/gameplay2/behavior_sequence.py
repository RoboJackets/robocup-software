from behavior import *


# A behavior sequence takes a list of behaviors and executes them in sequence.
# If one of these sub-behaviors fails, then the sequence fails and doesn't execute anything more
# The sequence moves onto the next behavior as soon as the current behavior completes
class BehaviorSequence(Behavior):

    def __init__(self, behaviors):
        continuous = behaviors[-1].is_continuous
        super().__init__(continuous)

        if behaviors == None or len(behaviors) == 0:
            raise RuntimeError("BehaviorSequence must be given one or more sub-behaviors")
        for bhvr in behaviors:
            if not bhvr.is_in_state(Behavior.State.start):
                raise RuntimeError("Attempt to initialize BehaviorSequence with behaviors not in the start state")

        # FIXME: make sure there's no continuous behaviors before the end

        self._behaviors = behaviors
        self._current_behavior_index = 0


    def execute_running(self):
        if not self.current_behavior.is_done_running():
            self.current_behavior.run()
        else:
            if self.current_behavior.is_in_state(Behavior.State.failed):
                self.transition(Behavior.State.failed)

                # cancel all subsequent behaviors
                for i in range(self.current_behavior_index + 1, len(self.behaviors)):
                    self.behaviors[i].terminate()

                # FIXME: cancel all subsequent behaviors
            elif self.current_behavior.is_in_state(Behavior.State.completed):
                self._current_behavior_index += 1
                if self._current_behavior_index >= len(self.behaviors):
                    self.transition(Behavior.State.completed)


    def terminate(self):
        if self.is_done_running():
            logging.warn("Attempt to terminate behavior that's already done running")
        else:
            first_to_cancel = self.current_behavior_index
            if self.current_behavior != None and self.current_behavior.is_done_running():
                first_to_cancel += 1
            for i in range(first_to_cancel, len(self.behaviors)):
                self.behaviors[i].terminate()

            self.transition(Behavior.State.cancelled)


    @property
    def behaviors(self):
        return self._behaviors


    @property
    def current_behavior_index(self):
        return self._current_behavior_index


    @property
    def current_behavior(self):
        if self.is_done_running() or self.state == Behavior.State.start:
            return None
        else:
            return self.behaviors[self.current_behavior_index]


    # # when the sequence's robot gets set, we need to set it for all sub-behaviors
    # @Behavior.robot.setter
    # def robot(self, value):
    #     self._robot = value
    #     for bhvr in self.behaviors:
    #         bhvr.robot = value


    def __str__(self):
        s = super().__str__()

        for bhvr in self.behaviors:
            s += "\n";
            s += "\t* " if bhvr == self.current_behavior else "\t  "
            s += str(bhvr).replace("\t", "\t\t")

        return s
