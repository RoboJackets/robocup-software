
# Motion Control

## Intro

Motion Control encompasses the tasks related to moving physical robots on the field.


## Computer-side

The following classes play a role in motion control on the computer:

* MotionConstraints
* OurRobot
* MotionControl
* Pid



Here's a basic outline of how the motion control system currently works:

* `Processor.run()`
	* `GameplayModule.run()`
		* `Play.run()`
			* `Behavior.run()`
				* `OurRobot.move(2dpoint)`
					* sets `targetPos` in `OurRobot._motionConstraints`
		* `OurRobot.replanIfNeeded()`
			* looks at _motionConstraints and uses RRTPlanner to generate a new plan if needed
	* `OurRobot.motionControl().run()`
		* sets body_x, body_y, and body_w on the robot's `RadioTx::Robot` packet
	* `Processor.sendRadioData()`
		* builds the RadioTx packet by conglomerating each `OurRobot`'s `RadioTx::Robot` packet



## Robot-side

The robots receive instructions from the field computer in the form of protobuf packets sent over the radio.  See control.{h, c} to see how the motion control code works.  The control code is called from the main() runloop.
