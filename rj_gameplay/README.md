# rj\_gameplay - Gameplay for RJ RoboCup SSL 

This is the Python library that controls gameplay, or high-level
decision-making. 

## Main Folders

**Infrastructure - `stp/`**

Stable code for gameplay.

**Implementation - `rj_gameplay/`**

Frequently changing code of gameplay. Subclasses the interfaces and abstract classes in stp/. (Read the docstrings for the relevant superclasses if ever you are confused.)

NOTE: for now, both the old and new infrastructure are existing simultaneously while we transfer over. To figure out if a file is old or new, look at the superclass in the class definition. Old classes subclass an interface (e.g. IPlay, ITactic, ISkill) and new classes subclass an abstract class (e.g. Play, Tactic, Role, Skill).

**Tests - `tests/stp`**

Unit tests for stp/.

## SRTP Framework

See [design doc](https://docs.google.com/document/d/1gRAF--W7FwGoof0--1l_pyjM-N4RmMiV6FhBvyQIRgM/edit?usp=sharing) for more detail.

**Skill - `rj_gameplay/skill/`**

Atomic robot behavior. (e.g. Receive, Move, Kick)

**Role - `rj_gameplay/role/`**

Complex single-robot behavior. (e.g. Goalie, Passer, Receiver)

**Tactic - `rj_gameplay/tactic/`**

Coordinates one or more Roles at a high level. Generates Role Requests. (e.g. PassTactic, WallTactic)

**Play - `rj_gameplay/play/`**

Coordinates all robots on the field for a given situation. Gets role requests from its tactics, then assigns them according to the cost functions of those role requests. (Examples: Basic122, Basic Defense)

The general flow of how a RobotIntent (or specific command for the robot) is created as follows:

GameplayNode -> SituationAnalyzer -> Play (selected based on situation) -> Tactic(s) -> Role(s) -> Skill(s)

The RobotIntent for a given robot is then passed back up the chain to GameplayNode, and sent via ROS to the C++ stack.

## Important Files

**Gameplay Node - `rj_gameplay/gameplay_node.py`**

ROS node that links gameplay to the C++ side of our codebase. Gets world\_state
and status of all robots, and sends motion commands to ROS.  (This second
function will be replaced by ROS Actions soon.)

Uses SituationAnalyzer to select the best play to run, calling tick() on the
play to get the list of skills, then ticking all of the resulting skills.

All currently working plays are in the import at the top of `gameplay_node.py`. Change the `test_play` variable at the bottom of this file to run a test play.

**Basic Play Selector - `stp/basic_play_selector.py`**

One-to-one mapping of the situation from SituationAnalyzer to a play to run.
Simple Python dict at the top of this file achieves that. (We would like to add
multiple play options per situation, in a cost function manner of course.)

**Global Params - `stp/global_parameters.py`**

Client for the C++-side global parameter server. Allows us to share constants
between C++ nodes and gameplay. After creating this class, global parameters
will be available as globals in the global\_parameters module.

**Local Params - `stp/local_parameters.py`**

Python-side params exist here. Allows sharing constants between different
gameplay files. Both param files unused as of right now.

**RC.py & Tests - `stp/rc.py`, `stp/testing.py`**

Common data structures (e.g. Robot, Ball, WorldState) and their unit tests.
