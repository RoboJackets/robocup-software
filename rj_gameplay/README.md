# stp - Gameplay for RJ RoboCup SSL
This is a library containing the rewrite of the RJ RoboCup gameplay library.

## High-Level
```
`stp/`         - infrastructure for RC gameplay
`rj_gameplay/` - implementation of RC gameplay
`tests/stp`    - unit tests for stp/
```

Our gameplay library adapts the [STP framework](https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.61.1972&rep=rep1&type=pdf) proposed by CMDragons:
```
Skill  - atomic robot behavior, behavior tree + Python wrapper for ROS action\*(e.g. Move, Kick)
Tactic - one single role in a play, which handles complex single robot behavior (e.g. Goalie, Receiver, Waller)
Play   - multi-robot behavior, gets potential roles from a list of at least 6 tactics and assigns them as it sees fit (e.g. Basic Defense)
```
\*in-progress

## More detail

```
`rj_gameplay/gameplay_node.py` - ROS node that links gameplay to the C++ side of our codebase. Gets world_state and status of all robots, sends motion commands to ROS. (This second function will be replaced by Action Server once we have it.)

`stp/coordinator.py`           - The coordinator is responsible for using SituationAnalyzer to select the best play to run, calling tick() on the play to get the list of skills, then ticking all of the resulting skills.

`stp/basic_play_selector.py`   - One-to-one mapping of the situation from SituationAnalyzer to a play to run. (Simple Python dict at the top of this file achieves that.)

`stp/global_parameters.py`     - A client for the C++-side global parameter server. Allows us to share constants between C++ nodes and gameplay. After creating this class, global parameters will be available as globals in the global_parameters module.

`stp/local_parameters.py`      - Python-side params exist here. Allows sharing constants between different gameplay files.

`stp/rc.py`                    - Data structures for RoboCup (e.g. Robot, Ball, WorldState)

`stp/testing.py`               - Testing `rc.py`'s data structures.
```
