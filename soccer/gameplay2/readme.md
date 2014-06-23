
# Python high-level soccer interface

This folder will contain the files needed to build a python module that will allow the lower-level parts of soccer such as the path-planner, network modules, etc to be controlled through python.  It also has all of our python code that depends on this module.

The main benefits of this are:

* python is much friendlier than c++, which will make our high-level code a lot more accessible to new members
* the play- and behavior-development cycles will be dramatically faster. rather than changing the c++ code for a play, exiting soccer, compiling for a couple minutes, relaunching, reconfiguring, then running, we'll be able to instantly reload a play whenever we detect a file change on disk. this will be huge for play development
* our high-level stuff needs some rethought anyways and this will be a good chance to revisit things such as the double touch rule and a setup phase for plays


## How it works

We are using the boost python library to develop a python module that allows python to interface with the system.  This consists of creating "wrappers" for C++ classes that tell python what methods are available, etc.  When soccer loads, it will setup an embedded python interpreter that will run the high-level code.  The C++ and python sides will communicate with eachother through some callbacks and shared variables.

Some classes that have python wrappers:

* Robot and OurRobot
* Point
* MotionConstraints
* GameState


Some classes that only exist in python land:

* Play
* Behavior
* PassingContext


## STP

The idea of separating high-level logic in to Skills, Tactics, and Plays is borrowed from [CMU](http://www.cs.cmu.edu/~mmv/papers/05STP-IEEE.pdf).

Skills are low-level, single-robot actions that should be highly-reuseable.

Tactics are higher-level and may combine Skills and other Tactics as sub-behaviors.  They can control one robot or multiple.  These should also be written to be reuseable by Plays and possibly other Tactics.

Plays coordinate the whole team of robots (minus the goalie) by combining Skills and Tactics.  Only one Play runs at a time.


## IMPORTANT

Not all python code is auto-reloaded, only the stuff in the 'skills', 'plays', 'tactics', and 'evaluation' folders (and their subfolders).  There are a few rules that you have to follow in order for the auto-reloading and play-registry systems to work.

Note for Sublime Text users: by default, sublime saves files atomically by editing a copy, then swapping it out for the original when it's "saved".  This screws up the `watchdog` python package we're using to watch for filesystem events.  You can fix this by disabling this feature in your sublime settings: Preferences -> User: `"atomic_save": false`.


* Only one play can be defined for play file (you can put as many skills as you want in a skill file though)
* Don't use `from xxxx import yyyy` where yyyy is not a module, use `import xxxx`.  This has to do with how python handles modules and the classes loaded from those modules.  If you do it the wrong way, when a play file changes on disk and we reload it as a module, the `yyyy` that you imported from the module earlier won't get reloaded.
* Each subdirectory that is autoloaded must have an empty \_\_init\_\_.py file so that python recognizes it as a package.  For example, if you add a new folder 'offense' in the 'plays' folder, you must put an \_\_init\_\_.py there.
* Because of the way the `imp.reload()` function works in python, you CAN NOT change the name of a play class while soccer is running.  Change its code, not its name.
* Unloading a module that has previously been imported is not supported in python.  This means that if you delete the file for a skill, the program will continue running as if nothing has changed.  If you delete a play, it will be removed from the play registry and will not be run anymore (although the module will still be loaded in memory).



# Common Errors

## No module named 'robocup'

The 'robocup' module is compiled from the robocup-py.cpp source file.  It is compiled as part of the `soccer` executable and passed into the python interpreter at runtime.  Thus, any python code loaded by `soccer` is able to `import robocup` and it'll work.  Python code that runs independent of `soccer`, has to link against the standalone module, which is placed at `run/robocup.so`.  In order for python to see it though, you'll have to make sure to add the run folder to the PYTHONPATH variable.
