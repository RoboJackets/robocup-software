import composite_behavior
import behavior


# A behavior sequence takes a list of behaviors and executes them in sequence.
# If one of these sub-behaviors fails, then the sequence fails and doesn't execute anything more
# The sequence moves onto the next behavior as soon as the current behavior completes
class BehaviorSequence(composite_behavior.CompositeBehavior):
    def __init__(self, behaviors= []):
        super().__init__(
            continuous=True
        )  # Note: we don't know if the sequence will be continuous or not, so we assume it is to be safe

        self.behaviors = behaviors

        self._current_behavior_index = -1

        self.add_transition(
            behavior.Behavior.State.start, behavior.Behavior.State.running,
            lambda: len(self.behaviors) > 0, 'has subbehavior sequence')

        self.add_transition(
            behavior.Behavior.State.running, behavior.Behavior.State.completed,
            lambda: self._current_behavior_index >= len(self.behaviors),
            'all subbehaviors complete')

        self.add_transition(
            behavior.Behavior.State.running, behavior.Behavior.State.failed,
            lambda: self.current_behavior != None and self.current_behavior.is_in_state(behavior.Behavior.State.failed),
            'subbehavior failed')

    def on_enter_start(self):
        # reset
        self._current_behavior_index = -1

    def on_enter_failed(self):
        self._terminate_subbehaviors()

    ## While we haven't gone through every behavior in the sequence, we
    #  Execute each individual behavior. When it finishes we increment
    #  the current_behavior index to the next behavior
    def execute_running(self):
        if self.should_advance():
            if self.current_behavior != None:
                self.remove_subbehavior('current')

            self._current_behavior_index += 1
            if self.current_behavior_index < len(self.behaviors):
                self.add_subbehavior(
                    self.behaviors[self.current_behavior_index],
                    'current',
                    required=True)

    #advances if the current behavior is done running, or if
    def should_advance(self):
        if len(self.behaviors) == self.current_behavior_index:
            #All behaviors completed
            return False
        if len(self.behaviors) > 0 and self.current_behavior_index == -1:
            # start up our first behavior
            return True
        if self.current_behavior != None and self.current_behavior.is_done_running():
            # this behavior finished, move onto the next
            return True
        return False

    def _terminate_subbehaviors(self):
        self.remove_all_subbehaviors()
        if self.behaviors != None:
            # note: we really should only do this for the ones that haven't been run yet,
            #       but calling terminate() on something that's already done running doesn't
            #       hurt anything and I'm lazy...
            for bhvr in self.behaviors:
                bhvr.terminate()

    def on_enter_cancelled(self):
        self._terminate_subbehaviors()

    @property
    def behaviors(self):
        return self._behaviors

    @behaviors.setter
    def behaviors(self, value):
        self._behaviors = value
        self.transition(behavior.Behavior.State.start)

    @property
    def current_behavior_index(self):
        return self._current_behavior_index

    @property
    def current_behavior(self):
        if self.has_subbehavior_with_name('current'):
            return self.subbehavior_with_name('current')

    def __str__(self):
        desc = super().__str__()
        if self.state == behavior.Behavior.State.running:
            desc += "\n    executing " + str(self.current_behavior_index +
                                             1) + "/" + str(len(
                                                 self.behaviors))

        return desc
