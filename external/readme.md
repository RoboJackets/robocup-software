
# External Dependencies

This folder contains our external dependencies as git submodules.  These dependencies are referred to in our CMakeLists files with the CMake ExternalProject feature, which clones them into the build directory and compiles them depending on our needs.

By default, when you clone this repo, it doesn't initialize the submodules, so they remain as empty directories.  In order to pull them down, run:

~~~{.sh}
git submodule update --init
~~~

To add a new submodule here, cd into this folder, then run:

~~~{.sh}
git submodule add git://website.com/my_repo name_of_new_submodule
~~~
