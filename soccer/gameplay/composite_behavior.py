import behavior
import single_robot_behavior
import role_assignment
import traceback
import logging
import re
import sys
from typing import Callable, Dict, Union


## A composite behavior is one that has 0+ named subbehaviors
# this class has methods for making it easy to work with and manage subbehaviors
class CompositeBehavior(behavior.Behavior):
    def __init__(self, continuous: bool) -> None:
        super().__init__(continuous=continuous)
        self._subbehavior_info = {}  # type: Dict[str, Dict]

    # FIXME: what if a subbehavior of @bhvr is required, but this is not?
    # FIXME: how do priorities work?
    # FIXME: how do nested priorities work?
    def add_subbehavior(self,
                        bhvr: behavior.Behavior,
                        name: str,
                        required: bool=True,
                        priority: Union[int, Callable[[], int]]=100):
        if name in self._subbehavior_info:
            raise AssertionError("There's already a subbehavior with name: '" +
                                 name + "'")

        if isinstance(priority, int):
            priority_func = (lambda: priority)
        else:
            priority_func = priority

        self._subbehavior_info[name] = {
            'required': required,
            'priority': priority_func,
            'behavior': bhvr
        }

    def remove_subbehavior(self, name: str):
        del self._subbehavior_info[name]

    def has_subbehavior_with_name(self, name: str):
        return name in self._subbehavior_info

    def has_subbehaviors(self) -> bool:
        return len(self._subbehavior_info) > 0

    def subbehavior_with_name(self, name: str):
        return self._subbehavior_info[name]['behavior']

    def subbehaviors_by_name(self) -> Dict[str, Dict]:
        by_name = {}
        for name in self._subbehavior_info:
            by_name[name] = self._subbehavior_info[name]['behavior']
        return by_name

    def remove_all_subbehaviors(self):
        subbehaviorNames = list(self._subbehavior_info.keys())
        for name in subbehaviorNames:
            self.remove_subbehavior(name)

    ## Returns a list of all subbehaviors
    def all_subbehaviors(self) -> list:
        return [
            self._subbehavior_info[name]['behavior']
            for name in self._subbehavior_info
        ]

    def all_subbehaviors_completed(self) -> bool:
        return all(
            [bhvr.is_done_running() for bhvr in self.all_subbehaviors()])

    ## Override StateMachine.spin() so we can call spin() on subbehaviors
    def spin(self):
        super().spin()
        # spin each subbehavior
        for name in list(self._subbehavior_info.keys()):
            info = self._subbehavior_info[name]
            bhvr = info['behavior']

            # multi-robot behaviors always get spun
            # only spin single robot behaviors when they have a robot
            should_spin = True
            if isinstance(bhvr, single_robot_behavior.
                          SingleRobotBehavior) and bhvr.robot is None:
                should_spin = False

            # try executing the subbehavior
            # if it throws an exception, catch it and pass it to the exception handler, which subclasses can override
            if should_spin:
                try:
                    bhvr.spin()
                except:
                    exc = sys.exc_info()[0]
                    self.handle_subbehavior_exception(name, exc)

    ## Override point for exception handling
    # this is called whenever a subbehavior throws an exception during spin()
    # subclasses of CompositeBehavior can override this to perform custom actions, such as removing the offending subbehavior
    # the default implementation logs the exception and re-raises it
    def handle_subbehavior_exception(self, name, exception):
        # We only call this inside the above except
        #pylint: disable=misplaced-bare-raise
        logging.error("Exception occurred when spinning subbehavior named '" +
                      name + "': " + str(exception))
        traceback.print_exc()
        raise

    ## returns a tree of role_requirements
    def role_requirements(self):
        reqs = {}
        for name, info in self._subbehavior_info.items():
            r = info['behavior'].role_requirements()
            # r could be a RoleRequirements or a dict forming a subtree
            if isinstance(r, role_assignment.RoleRequirements):
                r.required = info['required']
                r.priority = info['priority']()
            # FIXME: should required and priority propogate if it's a tree?
            reqs[name] = r
        return reqs

    # assignments is a tree with the same structure as that returned by role_requirements()
    # the only difference is that leaf nodes are (RoleRequirements, OurRobot) tuples
    # instead of just RoleRequirements
    def assign_roles(self, assignments):
        for name, subtree in assignments.items():
            self.subbehavior_with_name(name).assign_roles(subtree)

    def __str__(self) -> str:
        desc = super().__str__()

        for name in self._subbehavior_info:
            bhvr = self._subbehavior_info[name]['behavior']
            # indent the subbehavior's description
            indent = '    '
            subdesc = str(bhvr)
            subdesc = "\n" + indent + re.sub(r'\n', '\n' + indent, subdesc)
            desc += subdesc

        return desc
