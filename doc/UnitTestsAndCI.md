
# Testing & Continuous Integration

When working on a large software project, it's very helpful to have a test suite in place that can be quickly run to verify that key components haven't been broken by new changes.  The test suite won't cover everything, but the more it covers, the better.

Each time you make tests to the codebase, you should run the test suite with `make tests` to ensure that you haven't broken anything that was previously working.  Additionally, you should add unit tests for your new code when appropriate.


## C++ Unit Tests

We use the [googletest](https://code.google.com/p/googletest/) unit testing platform for our C++ code.  See the `soccer/tests` directory for our current tests.  You can run these by running the `test-cpp` executable in the `run` folder after compiling the project.


## Python Unit Tests

We use the standard `unittest` module for testing our python code.  Our unit tests are located at `soccer/gameplay/tests` and can be run by running `./run_tests.sh` form the `soccer/gameplay` directory.


## Continuous Integration

We use a free continuous integration service called [CircleCi](http://circleci.com), which recompiles and retests our project everytime we push a new commit to our [GitHub repo](http://github.com/robojackets/robocup-software).  Circle-ci works by cloning our git repository into a Ubuntu [Docker](https://www.docker.com) Container (similar to a Virtual Machine) each time it receives a notification from GitHub that there are new commits.  It then looks at our circle config file [circle.yml](../circle.yml), which runs other scripts that build and test our code.  Robocup-Software uses [DoCIF](https://github.com/jgkamat/DoCIF) to manage it's docker-based CI.  Triggered by the circle.yml file, DoCIF builds a baseimage for the project (if any setup files changed), runs the tests listed in [config.docif](../config.docif), and sends it's results back to github to be displayed.  On pull requests, you can see the individual logs of any test by clicking 'details' on any check.

There is a build status icon on our GitHub project's main page README that is green when the master branch build succeeds and red when it fails.  This lets us quickly ensure that our project build hasn't been broken.  Our goal is to make sure the master branch always builds.  We push new code to a separate branch, then merge it only once we verify that it passes the tests on circeci.

## Coverage

For every build, a seperate test is run that checks the percentage of lines in our codebase that are run by tests.  The results of this are sent to [Coveralls](https://coveralls.io/github/RoboJackets/robocup-software) so we can read them easily.  In addition, the percent coverage of the master branch is displayed on the main github repository alongside the status badge.
