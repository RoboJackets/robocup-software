#!/bin/bash

# runs all the tests in the tests directory
# we prepend the run directory so that `import robocup` finds the `robocup.so` file located there
export PYTHONPATH=$(readlink -f "../../install/lib")
echo "PYTHONPATH: $PYTHONPATH"
python3 -m unittest discover -s tests
