# GT RoboJackets RoboCup SSL [![Build Status](https://travis-ci.org/RoboJackets/robocup-software.png?branch=master)](https://travis-ci.org/RoboJackets/robocup-software)


The Georgia Tech RoboJackets team competes in the annual RoboCup Small Size League (SSL) competition.  This repository contains all of the software that we run on the main field laptop and on our robots.  More information on how our software works can be found on our [documentation page](http://robojackets.github.io/robocup-software/), our [wiki](http://wiki.robojackets.org/w/RoboCup_Software) or on our [website](http://www.robojackets.org/).
Also, check out our [2014 qualification video](https://www.youtube.com/watch?v=H3F9HexPLT0) to see our robots in action! 

![Screenshot of the 'soccer' program](doc/images/soccer.png "Soccer")


## The Competition

The official [RoboCup site](http://robocupssl.cpe.ku.ac.th/) has more information on the competition.


### The Basics

The soccer game is played between two teams of six robots each on a field with overhead cameras.  The field vision computer processes images from the cameras and sends out (x,y) coordinates of the ball and the robots.  Each team has a laptop that accepts the data from the vision computer and uses it to strategize, then send commands to robots on the field.


## Project Layout

### soccer/

The soccer folder contains the code to build the 'soccer' program, which is the main program in control when running our robots.


### simulator/

Code for the RoboCup simulator is located here.  This allows us to quickly test our soccer strategy on the computer.  Keep in mind though that just because something works well in the simulator doesn't mean it'll be the same in real life.


### firmware/

The firmware folder contains the code that runs on the robot (soccer/robot) and on the radio base station.  See the firmware/robot [README](firmware/robot/README.md) for more info.


### common/

Code that's shared between the different programs in our project is stored here.


### run/

Compiled programs and some configuration files are stored here.


## Setup

Here's a quick guide to getting this RoboCup project setup on your computer.  We recommend and only provide directions for installing on Ubuntu Linux, although it shouldn't be too difficult to port to other operating systems.

1) Clone the repository

```
git clone git://github.com/RoboJackets/robocup-software
```


2) Install the necessary software

The ubuntu-setup script uses apt-get to install several packages that are required to build and use the project.

```
$ cd robocup-software
$ util/ubuntu-setup
```

3) Build the project

```
$ scons
```

[Scons](http://www.scons.org/) is a python-based build system that we use to compile the executables that make up our project.  After running scons, several programs will be placed in the **run** folder.  See the [soccer docs](http://robojackets.github.io/robocup-software/md_soccer_doc__soccer.html) for instructions on running the soccer program.


## Documentation

We use [Doxygen](www.doxygen.org) for documentation.  This allows us to convert specially-formatted comments within code files into a nifty website that lets us easily see how things are laid out.  Our compiled doxygen documentation can be found here:

http://robojackets.github.io/robocup-software/

Note: The doxygen documentation site above is currently updated manually (pushing new code to master doesn't update our documentation site).  Checkout the `gh-pages` branch and take a look at the README to see how this is handled.


## Testing

We use [gtest](https://code.google.com/p/googletest/) for unit-testing our software, which can be run by running `scons test`.  To add a test to be run with the rest of the bunch, add it to the test sources variable in [test/SConscript](test/SConscript).
