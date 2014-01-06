
# gameplay

This folder contains the higher-level strategy code for the soccer program.


## GamePlayModule

The [GamePlayModule](./GamePlayModule.hpp) is the coordinator for all game logic.  Its main responsibilities include:

* managing the Goalie
* managing the joystick-controlled robot (if any)
* maintaining a list of global field obstacles
* choosing which Play to run
* running the current play


## Creating a Play

In order to add a custom play to the system, you must make a new play class, and then register the play and add it to the appropriate build lists. The steps are as follows:


### Create a play class

Copy the ExamplePlay class (both hpp and cpp files) with your new play class name. Change all occurrences of ExamplePlay.

Play classes go in `soccer/gameplay/plays`, and demo-only plays should go in `soccer/gameplay/plays/demo_plays`.


### Register the Play

You may assign the play to a category in the play list. Use REGISTER_PLAY_CATEGORY instead of REGISTER_PLAY.  This should be called in the .cpp file.

Note: Only one play class can be present in a source file. This is a limitation of REGISTER_PLAY, and if it really bothers you, it is possible to work around it (see the macro definition).


### Add the source files to scons

Add an entry for the new .ccp file to [soccer/SConscript](../SConscript) in the `srcs` array.

The play should now be available in the play selection tab of the soccer user interface, and can be enabled or disabled like the other plays.
