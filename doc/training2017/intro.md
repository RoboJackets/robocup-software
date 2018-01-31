# Training Summary 1 {#t20171}


# Download VM Image

-   [Download Link](https://mega.nz/#!958GxYrB!CIVIHgDFbNXLgfjgxrVmWa8ZkVOvhaXRP_x48-dUO8s) (<http://bit.ly/2yul2Qv>)
-   Today we will be setting you up to develop RoboCup Software


# Introductions

-   Name
-   Major/How Many Years at Tech
-   Programming experience


# Communication

-   Email List
-   Gitter (badge on our repo)
-   Docs Site (link on our repo)
-   Slack (robojackets.slack.com)


# Developing RoboCup

-   Running on Ubuntu 16.04
-   Recommended:
    -   Install our VM
    -   This comes with all dependencies already installed.
-   Supported:
    -   Install on your pre-existing 16.04 Installation (dual boot/VM)
    -   Anyone doing this?
-   Unsupported, needs a maintainer:
    -   Mac OS


# Create a GitHub Account

-   [https://github.com/join](https://github.com/join%0A)
-   Please include *at least* your real first name on github

![img](https://i.imgur.com/0cdXQXW.png)


# Fork Our Repo

-   RoboJackets/robocup-software (<http://bit.ly/1L70JbB>)

![img](https://i.imgur.com/kYzz2oh.png)


# RoboCup VM Setup

-   [Link to Guide](https://github.com/RoboJackets/robojackets-images/blob/master/robocup/USAGE.org) (<http://bit.ly/2chLRLd>)


## RoboCup Images Usage Guide

-   This document will help you get started using the RoboCup Software Virtualbox Image


### Install VirtualBox

-   Install VirtualBox from the [VirtualBox downloads page](https://www.virtualbox.org/wiki/Downloads).


### Ensure you have Virtualization turned on in your BIOS

-   [This](http://www.howtogeek.com/213795/how-to-enable-intel-vt-x-in-your-computers-bios-or-uefi-firmware/) is a simple guide of how to do this.
-   While this is not 100% necessary, it will make your VM much faster.
-   On a Windows host, you may need to turn off Hyper-V as well.


### Get a Copy of the built Image

-   First start by downloading the prebuilt image (a link should have been distributed to you) or by manually building the image.

-   See [the README](README.md) for more information about building this project.


### Extract Image

-   If you have downloaded the image from a distributed link, simply extract the zip file, which will produce several result files.

-   If you have manually extracted the image, go to `box/virtualbox` and extract the `.box` file there.
-   It can be extracted via tar, ex: `tar -xvf ubuntu1604-desktop-nocm-0.1.0.box`.


### Import Image into VBox

1.  Go to `File->Import Appliance`
    
    ![img](https://i.imgur.com/keQmMy4.png)

2.  Select the `.box` file you extracted earlier
    
    ![img](https://i.imgur.com/3S2Pgt3.png)

3.  Increase the Amount of Memory and CPU's
    
    Increase the Memory/CPU to your computer's specs. Don't allocate too much memory/cpus!
    
    ![img](https://i.imgur.com/P8Adm2a.png)

4.  Hit `Import`!


### Configure Settings of Imported Image

1.  Right click your new virtualbox entry, and hit `settings`

2.  Increase the Amount of Video RAM, and turn on 3D Acceleration
    
    If you do not have virtualization, virtualbox may not allow you to turn on 3D Acceleration
    
    ![img](https://i.imgur.com/YzmNmcM.png)

3.  Turn **OFF** `Remote Display`
    
    ![img](https://i.imgur.com/cvigW2G.png)


### Boot your new VM

-   Double Click the Entry, or Right Click -> Start -> Normal Start


### Credentials

| User    | Password |
|------- |-------- |
| vagrant | vagrant  |

The sudo password is `vagrant`.


### Testing

To test out your new RoboCup VM, open a terminal (use the search in the top left), and type these two commands

```shell
cd ~/robocup-software
make run-sim
```


### You Made It!

-   Have a great time contributing to the largest and most prestigious undergraduate, student-run, autonomous soccer codebase at Georgia Tech.


# PostInstall Configuration


## Git Config

-   These commands set up git to attribute your changes to you.
    
    ```shell
    git config --global user.name "Your Name"
    git config --global user.email "your@email.here"
    ```


## Adding Remotes

-   These commands set you up to push to your fork by default, and still be able to receive updates.

```shell
cd ~/robocup-software
git remote add rj https://github.com/RoboJackets/robocup-software.git
git remote set-url origin https://github.com/<YOUR_GH_USERNAME>/robocup-software.git
git remote -v

# You can find the link in the last command under the 'clone or download'
# green button on your FORK
```


## Build RoboCup Software

```shell
cd ~/robocup-software
make

# Try running robocup-software by running this:
make run-sim
```


## Staying Updated

```shell
git pull rj master
git push origin master
```


# Soccer Tutorial

-   'Soccer' is our main program.


## Helpful Build Commands

| Command        | Description                          |
|-------------- |------------------------------------ |
| make           | Compiles rc-software                 |
| make run       | Run Soccer with no simulator         |
| make run-sim   | Run Soccer with a simulator          |
| make debug     | Run Soccer in a debugger             |
| make debug-sim | Run Soccer in a debugger + simulator |


## In Depth Tutorial


### Soccer Image

![img](https://i.imgur.com/vgDnpjo.png)

-   To start this tutorial, run `make run-sim` in the root of your project.


### Running a Play

1.  Select a play from the plays menu in the lower right (try `RepeatedLineUp` for now).
2.  Click the green `Force Start` check mark in the upper left.
3.  This should start the play.
4.  To stop the play (for debugging or other reaons), click the `Halt` hand in the upper left.


### Behavior Tree

In the upper right, you will see a 'behavior tree' box. This is where important information about the plays you are running shows up.

To try it out, run a play, and keep a close eye on the behavior tree box. You should see a tree of plays and their properties (which helps to identify where exactly something is going wrong).


### Backtracing Through Logs

While in this section, try looking at the behavior tree while different plays run, and try to get an idea of what's going on!

1.  Try running a play, then press the pause icon in the top center of the screen. This will cause your view of the robots to stop moving, but in reality, soccer is still simulating in the background.
2.  Press the `Halt` hand in the upper left to stop the robots in reality (to conserve resources while you play around)
3.  Pressing the arrows immediately to the right and left of the pause icon step forward/backward in time by a single frame.
4.  Pressing the Arrows outside of the single step arrows move time slowly in that respective direction.
5.  You can use the log slider above all these buttons to move coarsely to a speicific time.
6.  Pressing the live button to the right of the pause button resets the view to the current time.


### Goalie/Manual Selectors

-   The goalie selector in the top right selects our current goalie. This is needed because RoboCup requires we can only use one specific robot for a goalie at a time.
-   The Manual selector is used for manually controlling a robot. Select the robot's radio id in this selector and use a joystick to manually control a robot.


### Additional Notes

-   Soccer is a very complicated program, but thankfully it is fairly intuitive. Play around with it and see what different options do!