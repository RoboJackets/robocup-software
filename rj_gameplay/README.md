# rj\_gameplay - Gameplay for RJ RoboCup SSL 

This is the Python library that controls gameplay, or high-level
decision-making. For a more detailed overview, see this [Google Doc](https://docs.google.com/document/d/1pf-aaHnvonTPC6ccI7RdHudpuev84fZslMVDq8lw59s/edit?usp=sharing).

## Main Folders

**Infrastructure - `stp/`**

Stable code for gameplay.

**Implementation - `rj_gameplay/`**

Frequently changing code of gameplay.

**Tests - `tests/stp`**

Unit tests for stp/.

## STP Framework

Our gameplay library adopts the name of the [STP
framework](https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.61.1972&rep=rep1&type=pdf)
proposed by CMDragons, but is quite different. All implementation files for STP are in `rj_gameplay/` as clarified above.

**Play - `rj_gameplay/play/`**

Handles full team behavior. Gets potential roles from a list of >=6 tactics,
then assigns 6 of them at any time.

(Examples: `basic_defense.py` in folder above)

**Tactic - `rj_gameplay/tactic/`**

Handles role costing for one or more robots, such that Plays can choose
higher-level ideas for each robot. Think of as abstraction layer between single-robot
Skills and whole-team Plays.

(Examples: `goalie_tactic.py, wall_tactic.py, pass_tactic.py` in folder above)

**Skill - `rj_gameplay/skill/`**

Atomic robot behavior which consists of a behavior tree of actions.

(Examples: `move.py, mark.py, capture.py` in folder above)

**Action - `rj_gameplay/action/`** (in-progress)

ROS Action Client that sends an action request to C++-side Action Server. This
then handles motion planning & control and sends feedback to the client. Note
that "ROS Action" =/= "Action" in the context of gameplay.

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
