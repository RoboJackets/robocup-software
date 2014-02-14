
# Gameplay

The Gameplay namespace contains most of the higher-level logic of the `soccer` program.


Gameplay::GameplayModule



## Play Structure

The high-level strategy code is organized to be as modular as possible.  To do this, it's been split up into three main parts: **Behaviors**, **Tactics**, and **Plays**.  The GameplayModule has one Goalie (optionally) and one Play.

\dot
digraph play_structure {
    subgraph cluster_gameplay {
        label = "GameplayModule";
        node [shape = record];
        Gameplay [label = "{self|{<r0>|<r1>|<r2>|<r3>|<r4>|<r5>}}"];
    }

    Robot0, Robot1, Robot2, Robot3, Robot4, Robot5 [label = "Robot", shape=ellipse];

    node [shape = box];
    Gameplay:r0 -> Goalie;
    Goalie -> Robot0;

    Gameplay:r1 -> Play;
    Gameplay:r2 -> Play;
    Gameplay:r3 -> Play;
    Gameplay:r4 -> Play;
    Gameplay:r5 -> Play;

    node [shape=record];
    Behavior1, Behavior2, Behavior3 [label="Behavior"];
    Behavior4 [label="{Behavior|{<r3>|<r4>}}"];

    Play -> Behavior1 -> Robot1;
    Play -> Behavior2 -> Robot2;
    Play -> Behavior3 -> Robot3;
    Play -> Behavior4;
    Behavior4:r3 -> Robot4;
    Behavior4:r4 -> Robot5;
}
\enddot


### Behavior

Behaviors are simple actions.  A few examples are:

* **Gameplay::Behaviors::Move** - navigates the robot to a specified (x,y) coordinate on the field
* **Gameplay::Behaviors::LineKick** - given a target line segment to hit, it lines the robot up behind the ball, approaches it, then kicks it towards the target
* **Gameplay::Behaviors::Capture** - drives the robot towards the ball, turns the dribbler on, and attempts to bring the ball under its posession


### Tactic

Currently our distinction between Behaviors and Tactics isn't overly clear...  The code probably needs some cleanup here.


### Play

Plays are the highest-level actions and there is only one active at any given time.  Plays are responsible for coordinating actions amongst all available robots.  Plays are generally made up of Behaviors and Tactics.  A few example plays are:

* Gameplay::Plays::OurFreekick
* Gameplay::Plays::TheirFreekick
* Gameplay::Plays::OurCornerKick
* Gameplay::Plays::OurGoalKick
* Gameplay::Plays::Stopped
* Gameplay::Plays::MightyMight - our main offensive play


## Creating a Play

In order to add a custom play to the system, you must make a new play class, and then register the play and add it to the appropriate build lists. The steps are as follows:


### Create a subclass of Play

Copy the Gameplay::Plays::ExamplePlay class (both hpp and cpp files) with your new play class name. Change all occurrences of ExamplePlay.

Play classes go in `soccer/gameplay/plays`, and demo-only plays should go in `soccer/gameplay/plays/demo_plays`.


### Register the Play

You may assign the play to a category in the play list. Use REGISTER_PLAY_CATEGORY instead of REGISTER_PLAY.  This should be called in the .cpp file.

Note: Only one play class can be present in a source file. This is a limitation of REGISTER_PLAY, and if it really bothers you, it is possible to work around it (see the macro definition).


### Add the source files to scons

Add an entry for the new .ccp file to `soccer/SConscript` in the `srcs` array.

The play should now be available in the play selection tab of the soccer user interface, and can be enabled or disabled like the other plays.
