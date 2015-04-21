
# Build Systems

A build system or tool is a program that is responsible for compiling code into the desired output programs and libraries, as well as automating certain tasks.  Rather than executing the compiler directly, you tell the build system what files you'd like compiled and let it handle the compiling.


## CMake

We use [CMake](http://www.cmake.org/) as our main build system.  This is configured using CMakeLists.txt files spread throughout our project.  CMake works by turning the CMakeLists.txt files into a set of makefiles, which are then executed by the `make` program.

To build the project using CMake, you could do:

1. `mkdir build`
2. `cd build`
3. `cmake ..`
4. `make`

Assuming everything ran successfully, this would place `soccer`, `simulator`, and our other targets in the output folder `run`.

Rather than executing these commands each time we want to rebuild the project, we've added a shortcut using a `makefile`.  This makefile specifies the above set of commands under the default target, so you can now just run `make` in the root directory to build everything.  This makefile also has targets for a few other things.  Open it up in a text editor if you're curious.


## Scons

[Scons](http://scons.org) is another build system that we use.  It serves a similar purpose to CMake, but is based on the python programming language and rather than CMakeLists.txt files, Scons uses SConscript files to specify builds.  We use Scons to build our robot and radio base station firmware.  We could fairly easily port this to CMake too, but as of now it's still written with Scons.
