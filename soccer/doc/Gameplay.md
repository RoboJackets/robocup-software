
# Gameplay

This document covers the basics of our high-level soccer code including Plays, Behaviors, and game state evaluation.  This code resides in [soccer/gameplay](https://github.com/RoboJackets/robocup-software/tree/master/soccer/gameplay).



## Play Structure

The high-level strategy code is organized to be as modular as possible.  To do this, it's been split up into three main parts: **Behaviors**, **Tactics**, and **Plays**.  There is one Goalie (optionally) and one Play.

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


### Skill

Skills are simple Behaviors that apply to a single robot and are instances of the SingleRobotBehavior class.


### Tactic

Tactics are Behaviors that apply to one or more robots and are higher-level than a Skill.  Note: there is no Tactic class, the distinction between it and other behaviors is mostly for organizational purposes.


### Play

Plays are the highest-level actions and there is only one active at any given time.  Plays are responsible for coordinating actions amongst all available robot and are generally made up of Behaviors and Tactics.


## Creating a Play

Making a new play is as simple as adding a new python file somewhere inside the `soccer/gameplay/plays` directory and writing a subclass of `Play` inside of it.  There is no need to register the play, soccer will see the file in that folder and display it in the Play Config Tab.
