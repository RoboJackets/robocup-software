"""This module contains the implementation of the coordinator."""
from typing import Any, Dict, Optional, Type, List

import stp.play
import stp.rc as rc
import stp.role.assignment as assignment
import stp.situation
from rj_msgs import msg
from rj_geometry_msgs.msg import Point

NUM_ROBOTS = 16

class Coordinator:
    """The coordinator is responsible for using SituationAnalyzer to select the best
    play to run, calling tick() on the play to get the list of skills, then ticking
    all of the resulting skills."""

    __slots__ = [
        "_play_registry",
        "_play_selector",
        "_prev_situation",
        "_prev_play",
        "_prev_role_results",
        "_props",
    ]

    _play_selector: stp.situation.IPlaySelector
    _prev_situation: Optional[stp.situation.ISituation]
    _prev_play: Optional[stp.play.IPlay]
    _prev_role_results: assignment.FlatRoleResults
    _props: Dict[Type[stp.play.IPlay], Any]

    # TODO(1585): Properly handle type annotations for props instead of using Any.

    def __init__(self, play_selector: stp.situation.IPlaySelector):
        self._play_selector = play_selector
        self._props = {}
        self._prev_situation = None
        self._prev_play = None
        self._prev_role_results = {}

    def tick(self, world_state: rc.WorldState) -> List[msg.RobotIntent]:
        """Performs 1 ticks of the STP system:
            1. Selects the best play to run given the passed in world state.
            2. Ticks the best play, collecting the list of skills to run.
            3. Ticks the list of skills.
        :param world_state: The current state of the world.
        """

        # Call situational analysis to see which play should be running.
        cur_situation, cur_play = self._play_selector.select(world_state)

        cur_play_type: Type[stp.play.IPlay] = type(cur_play)

        # Update the props.
        cur_play_props = cur_play.compute_props(self._props.get(cur_play_type, None))

        if type(cur_play) == type(self._prev_play) and not self._prev_play.is_done(world_state):
            cur_play = self._prev_play
            # This should be checked here or in the play selector, so we can restart a play easily

        # Collect the list of skills from the play.
        new_role_results, skills = cur_play.tick(
            world_state, self._prev_role_results, cur_play_props
        )
    
        # Get the list of actions from the skills
        actions = {}
        for skill in skills:
            robot = new_role_results[skill][0].role.robot
            actions.update(skill.skill.tick(robot, world_state))
        intents = [msg.RobotIntent() for i in range(NUM_ROBOTS)]
        # intents = [msg.RobotIntent()] * NUM_ROBOTS
        # Get the list of robot intents from the actions
        for i in range(NUM_ROBOTS):
            if i in actions.keys() and actions[i]:
                for action in actions[i]:
                    intents[i] = action.tick(intents[i])
            else:
                intents[i].motion_command.empty_command = [msg.EmptyMotionCommand()]
        # pos = world_state.our_robots[1].pose[0:2]
        # pivot_command = msg.PivotMotionCommand()
        # pivot_command.pivot_point = Point(x=pos[0], y=pos[1])
        # pivot_command.pivot_target = Point(x=0.0, y=0.0)
        # intents[1] = msg.RobotIntent()
        # intents[1].motion_command.pivot_command = [pivot_command]
        # intents[1].dribbler_speed = 0.0
        # intents[1].is_active = True
        # Update _prev_*.
        self._prev_situation = cur_situation
        self._prev_play = cur_play
        self._prev_role_results = new_role_results
        self._props[cur_play_type] = cur_play_props

        return intents
