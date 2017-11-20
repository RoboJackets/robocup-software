# GT RoboJackets RoboCup SSL
[![16.04 Build Status](https://circleci.com/gh/RoboJackets/robocup-software.svg?&style=shield)](https://circleci.com/gh/RoboJackets/robocup-software) [![14.04 Build Status](https://semaphoreci.com/api/v1/jgkamat/robocup-software/branches/master/shields_badge.svg)](https://semaphoreci.com/jgkamat/robocup-software) [![Coverage Status](https://coveralls.io/repos/RoboJackets/robocup-software/badge.svg?branch=master&service=github)](https://coveralls.io/github/RoboJackets/robocup-software?branch=master)

[![Riot](https://img.shields.io/badge/matrix-riot%20chat-blue.svg)](https://riot.im/app/#/room/%23robocup-software:matrix.org) [![Join the chat at https://gitter.im/RoboJackets/robocup-software](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/RoboJackets/robocup-software?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)


The Georgia Tech RoboJackets team competes in the annual RoboCup Small Size League (SSL) competition.  This repository contains all of the software that we run on the main field laptop and on our robots.  More information on how our software works can be found on our [documentation page](http://robojackets.github.io/robocup-software/), our [wiki](http://wiki.robojackets.org/w/RoboCup_Software) or on our [website](http://www.robojackets.org/).
Also, check out our [2014 qualification video](https://www.youtube.com/watch?v=H3F9HexPLT0) to see our robots in action!

Here is a screenshot of our 'soccer' program:
![Screenshot of the 'soccer' program](doc/images/soccer.png "Soccer")


## The Competition

The soccer game is played between two teams of six robots each on a field with overhead cameras.  The field vision computer processes images from the cameras and sends out (x,y) coordinates of the ball and the robots.  Each team has a laptop that accepts the data from the vision computer and uses it to strategize, then send commands to robots on the field.

The official [RoboCup site](http://robocupssl.cpe.ku.ac.th/) has more information on the competition.


## Project Layout

`robocup-software` is split into 3 parts, [software](https://github.com/RoboJackets/robocup-software), [firmware](https://github.com/RoboJackets/robocup-firmware), and a [common](https://github.com/RoboJackets/robocup-common) part, used by both software and firmware. This repository contains the software portion of our codebase, the part that runs on our laptops, running high level plays.

### soccer/

The soccer folder contains the code to build the 'soccer' program, which is the main program in control when running our robots.


### common/

Code that's shared between the software and firmware sections of our codebase is stored here, as a [git submodule](https://git-scm.com/book/en/v2/Git-Tools-Submodules). See RoboJackets/robocup-common.

### external/

External dependencies that our code relys on, stored as git submodules. To initialize these, please run `git submodule update --init --recursive`.

### run/

Compiled programs and some configuration files are stored here.


## Setup

Here's a quick guide to getting this RoboCup project setup on your computer.  We recommend and only provide directions for installing on Ubuntu Linux and Arch Linux, although it shouldn't be too difficult to port to other operating systems.

1) Clone the repository

```sh
git clone git://github.com/RoboJackets/robocup-software
```


2) Install the necessary software

There are a few setup scripts in the util directory for installing required packages, setting up udev rules, etc.  See `ubuntu-setup`, `arch-setup`, and `osx-setup` for more info.

```sh
$ cd robocup-software
$ util/<SYSTEM>-setup
```

3) Build the project

```sh
$ make
```

We use CMake as our build system and have a simple `makefile` setup that invokes CMake.

After running `make`, several programs will be placed in the **run** folder.  See the [soccer docs](http://robojackets.github.io/robocup-software/md_soccer_doc__soccer.html) for instructions on running the soccer program.


## Documentation

We use [Doxygen](www.doxygen.org) for documentation.  This allows us to convert specially-formatted comments within code files into a nifty website that lets us easily see how things are laid out.  Our compiled doxygen documentation for software can be found here:

http://robojackets.github.io/robocup-software/

Note: The doxygen documentation site above is updated automacally using circle-ci.  See our autoupdate-docs.sh file for more info.

## Testing
We use [gtest](https://code.google.com/p/googletest/) for unit-testing our software, which can be run by running `make tests`.  To add a test to be run with the rest of the bunch, add a new file in soccer/tests.

The soccer tests can be run using `make test-soccer` or firmware tests with `make test-firmware`.
The TESTS name filter to run only certain tests. For example `make test-soccer TESTS=Point*` runs only the tests for the Point class.

## License

This project is licensed under the Apache License v2.0.  See the [LICENSE](LICENSE) file for more information.
