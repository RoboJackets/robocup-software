import composite_behavior as cb
import behavior
from typing import List
import logging


# A behavior sequence takes a list of behaviors and executes them in sequence.
# If one of these sub-behaviors fails, then the sequence fails and doesn't execute anything more
# The sequence moves onto the next behavior as soon as the current behavior completes
class BehaviorSequence(cb.CompositeBehavior):
    def __init__(self,
                 continuous=False,
                 repeat=False,
                 behaviors: List[cb.CompositeBehavior]=[]) -> None:
        super().__init__(continuous=continuous)

        self.behaviors = behaviors
        self.repeat = repeat

        self._current_behavior_index = -1

        self.add_transition(
            behavior.Behavior.State.start, behavior.Behavior.State.running,
            lambda: len(self.behaviors) > 0, 'has subbehavior sequence')

        self.add_transition(
            behavior.Behavior.State.running, behavior.Behavior.State.completed,
            lambda: (not repeat and self._current_behavior_index >= len(self.behaviors)),
            'all subbehaviors complete')

        self.add_transition(
            behavior.Behavior.State.running, behavior.Behavior.State.failed,
            lambda: (self.current_behavior() is not None and self.current_behavior().state == behavior.Behavior.State.failed),
            'subbehavior failed')

    def on_enter_start(self):
        # reset
        self._current_behavior_index = -1

    ## While we haven't gone through every behavior in the sequence, we
    #  Execute each individual behavior. When it finishes we increment
    #  the current_behavior index to the next behavior
    def execute_running(self):
        if self.should_advance():
            if self.current_behavior() is not None:
                self.remove_subbehavior('current')

            self._current_behavior_index += 1

            if self.current_behavior_index < len(self.behaviors):
                # restart the subbehavior in case we have run it before
                self.behaviors[self.current_behavior_index].restart()
                self.add_subbehavior(
                    self.behaviors[self.current_behavior_index],
                    'current',
                    required=True)
            elif self.repeat:
                logging.info("Restarting behavior sequence...")
                self.restart()

    #advances if the current behavior is done running, or if
    def should_advance(self):
        if len(self.behaviors) == self.current_behavior_index:
            #All behaviors completed
            return False
        if len(self.behaviors) > 0 and self.current_behavior_index == -1:
            # start up our first behavior
            return True
        if (self.current_behavior() is not None and
                self.current_behavior().is_done_running()):
            # this behavior finished, move onto the next, if we were successful
            return (not self.current_behavior ==
                    (behavior.Behavior.State.failed))
        return False

    def _terminate_subbehaviors(self):
        self.remove_all_subbehaviors()
        if self.behaviors is not None:
            for bhvr in \
                filter(lambda x: not x.is_done_running(), self.behaviors):
                bhvr.terminate()

    def on_exit_running(self):
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

    def current_behavior(self):
        if self.has_subbehavior_with_name('current'):
            return self.subbehavior_with_name('current')
        return None

    def __str__(self):
        desc = super().__str__()
        if self.state == behavior.Behavior.State.running:
            desc += ("\n    executing " + str(self.current_behavior_index + 1)
                     + "/" + str(len(self.behaviors)))
        return desc
