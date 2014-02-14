
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

The robots receive instructions from the field computer in the form of protobuf packets sent over the radio.  See control.{h, c} to see how the motion control code works.  The control code is called from the main() runloop.
