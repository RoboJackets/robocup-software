
# Motion Control

## Intro

Motion Control encompasses the tasks related to moving physical robots on the field.


## Computer-side

The following classes play a role in motion control on the computer:

* FaceTarget
* MotionTarget
* OurRobot
* MotionCommand
* MotionControl
* Pid

## Robot-side

The robots receive instructions from the field computer in the form of protobuf packets sent over the radio.  The spec for these can be found
