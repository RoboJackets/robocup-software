# robocup-fshared
Interface and shared files for connection between robocup-software and robocup-firmware

# Building
robocup-fshare tests and libs can be built standalone
```sh
mkdir build
cd build
cmake ..
make
```

This will create a static library called librc-fshare.a and currently
a git-version-test executable in the build directory. The git-version-test
binary can be run to check the git version output.

# How to use
This repo is set up to cleanly build a static library with all shared firmware code,
along with properly scoped includes.

All that should be needed is to call `target_link_libraries(<your_target> rc-fshare)`,
include paths and link commands should all propogate properly.

Then include headers from rc-fshare by using `#include "rc-fshare/pid.hpp"` for example.
