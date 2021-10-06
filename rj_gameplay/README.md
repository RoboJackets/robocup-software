# rj\_gameplay - Gameplay for RJ RoboCup SSL 

This is the Python library that controls gameplay, or high-level
decision-making. 

Main folders: 

**Infrastructure - `stp/`**

Stable code for gameplay

**Implementation - `rj_gameplay/`**

Frequently changing code of gameplay 

**Tests - `tests/stp`**

Unit tests for stp/ 

## STP Framework

Our gameplay library adapts the [STP
framework](https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.61.1972&rep=rep1&type=pdf)
proposed by CMDragons: 

**Skill - `rj_gameplay/skill/**

Atomic robot behavior which consists of a behavior tree and calls a ROS action.
Examples:
    - Move 
    - Kick

**Tactic - `rj_gameplay/tactic/**

Handles complex single robot behavior. Think of as one role in a play.
Examples:
    - Goalie
    - Receiver
    - Waller 

**Play - `rj_gameplay/play/**

Handles multi-robot behavior. Gets potential roles from a list of >=6 tactics,
then assigns them as it sees fit. Examples:
    - Basic Defense

## Important Files

**Gameplay Node - `rj_gameplay/gameplay_node.py`**

ROS node that links gameplay to the C++ side of our codebase. Gets world\_state
and status of all robots, and sends motion commands to ROS.  (This second
function will be replaced by ROS Actions soon.)

**Coordinator - `stp/coordinator.py`**

Uses SituationAnalyzer to select the best play to run, calling tick() on the
play to get the list of skills, then ticking all of the resulting skills.

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
