
# Python high-level soccer interface

This folder will contain the files needed to build a python module that will allow the lower-level parts of soccer such as the path-planner, network modules, etc to be controlled through python.

The main benefits of this are:

* python is much friendlier than c++, which will make our high-level code a lot more accessible to new members
* the play- and behavior-development cycles will be dramatically faster. rather than changing the c++ code for a play, exiting soccer, compiling for a couple minutes, relaunching, reconfiguring, then running, we'll be able to instantly reload a play whenever we detect a file change on disk. this will be huge for play development
* our high-level stuff needs some rethought anyways and this will be a good chance to revisit things such as the double touch rule and a setup phase for plays


## How it works

We are using the boost python library to develop a python module that allows python to interface with the system.  This consists of creating "wrappers" for C++ classes that tell python what methods are available, etc.  When soccer loads, it will setup an embedded python interpreter that will run the high-level code.  The C++ and python sides will communicate with eachother through some callbacks and shared variables.

Some classes that will need to be wrapped:

* Robot and OurRobot
* Point
* MotionConstraints
* GameState


Some classes that will only exist in python land:

* Play
* Behavior
* PassingContext



## IMPORTANT

Not all python code is auto-reloaded, only the stuff in the 'skills', 'plays', and 'tactics' folders (and their subfolders).  There are a few rules that you have to follow in order for the auto-reloading and play-registry systems to work.

* Only one play can be defined for play file (you can put as many skills as you want in a skill file though)
* Don't use `from xxxx import yyyy` where yyyy is not a module, use `import xxxx`.  This has to do with how python handles modules and the classes loaded from those modules.  If you do it the wrong way, when a play file changes on disk and we reload it as a module, the `yyyy` that you imported from the module earlier won't get reloaded.
* Each subdirectory that is autoloaded must have an empty __init__.py file so that python recognizes it as a package.  For example, if you add a new folder 'offense' in the 'plays' folder, you must put an __init__.py there.
