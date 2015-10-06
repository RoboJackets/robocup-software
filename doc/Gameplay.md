
# Gameplay

This document covers the basics of our high-level soccer code including Plays, Behaviors, and game state evaluation.  This code resides in [soccer/gameplay](https://github.com/RoboJackets/robocup-software/tree/master/soccer/gameplay).

As with any complex system, it's important to have some well-defined structure to keep things manageable.
The `soccer` program is split up into several different layers for this purpose, with the gameplay layer being the most high-level.
The gameplay layer is managed by the \ref GameplayModule and evaluates the current state of the field (where the robots and ball are) and the state of the game (is it a kickoff, penalty kick, etc).  This information is contained in the c++ \ref SystemState "SystemState" class and the \ref GameState "GameState class", respectively.
The result of running the \ref GameplayModule is a motion command for each of the robots as well as kick and dribble commands.  Layers of the software stack below the gameplay layer ideally don't know anything about soccer and just orchestrate robot motion, radio communication, network communication, etc.

When the gameplay module is running, its job is to select the best play from a list of enabled plays by choosing the one with the lowest \ref gameplay.play.Play.score() "score()" value.  Plays are enabled and disabled through the GUI with the checkboxes next to play names.  See the annotated screenshot below for more info.


![soccer screenshot with gameplay annotations](soccer-with-gameplay-annotations.png)

## Play Structure

The high-level strategy code is organized to be as modular as possible.
To do this, it's been split up into three main parts: [Skills](soccer/gameplay/skills), [Tactics](soccer/gameplay/tactics), and [Plays](soccer/gameplay/plays).
There is one Goalie (optionally) and one \ref gameplay.play.Play "Play" object.

**Skills** are behaviors that apply to a single robot.
They include things like \ref gameplay.skills.capture.Capture "capturing the ball", \ref gameplay.skills.move.Move "moving to a particular position on the field", and \ref gameplay.skills.pivot_kick.PivotKick "kicking the ball".

**Tactics** can coordinate a single robot or many and generally incapsulate more complex behavior than **skills**.  This includes things such as \ref gameplay.tactics.coordinated_pass.CoordinatedPass "passing", \ref gameplay.tactics.defense.Defense "defense", and \ref gameplay.tactics.positions.goalie.Goalie "the goalie".

**Plays** are responsible for coordinating the whole team of robots (although some robots may be unused).  At a given time, the `soccer` program is running at most one play.

Used together, skills, tactics, and plays form a tree structure with the Play at the root and other behaviors below it.  The C++ `GameplayModule` tells the current play to run, which in turn tells each of its sub-behaviors to run.

## Creating a Play

Making a new play is as simple as adding a new python file somewhere inside the `soccer/gameplay/plays` directory and creating a subclass of `Play` inside of it.
There is no need to register the play, soccer will see the file in that folder and display it in the Plays tab in `soccer`.
Generally when writing a new play, it's a good idea to base its initial structure on an existing play.  A good example to look at is the \ref gameplay.plays.testing.line_up "LineUp play".



TODO: brainstorming below

* lots of complexity, manage it by using STP and state machines
* state machines
  * states and substates
  * fsm.py
  * diagrams
    * show an example
    * show how to generate diagrams
  * declare state machine transitions and states in __init__()
* python vs c++
  * automatic reloading of plays and behaviors
* boost.python wrappers in robocup-py.cpp
  * robocup.so in `run` dir can be imported just like any other python module
* the gameplay layer is the only one that should know anything about soccer ideally
* the gameplay system results in a set of path-planning requirements for each robot and whether or not to kick, dribble

* robot assignment
  * don't typically set robots directly
  * The current play forms a tree of behaviors and each leaf has a corresponding role requirements object
  * [munkres/hungarian algorithm](https://en.wikipedia.org/wiki/Hungarian_algorithm)

* behavior tree in UI
  * single string stored on each LogFrame, so play/pause/rewind work fine with this

* debug drawing using SystemState

* evaluation
  * WindowEvaluator
  * pass eval?
  * ball prediction?
