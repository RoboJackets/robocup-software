
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



Here's a basic outline of how the motion control system currently works:

* `Processor.run()`
	* `GameplayModule.run()`
		* `Play.run()`
			* `Behavior.run()`
				* `OurRobot.move(2dpoint)`
					* sets value of `_delayed_goal`
		* `OurRobot.execute()`
			* feeds _delayed_goal to the RRT planner to replan (if necessary)
			* sets `cmd.target.pos` using `findGoalOnPath()`
			* `findGoalOnPath()` finds the point on `_path` closest to the robot's current position, then moves it along the path a bit so the robot follows the path
	* `OurRobot.motionControl().run()`
		* `positionPD()`
			* sets the velocities in the robot's MotionCommand
		* `anglePD()`
			* sets angularVelocity of robot's MotionCommand
		* sets body_x, body_y, and body_w on the robot's `RadioTx::Robot` packet
	* `Processor.sendRadioData()`
		* builds the RadioTx packet by conglomerating each `OurRobot`'s `RadioTx::Robot` packet



## Robot-side

The robots receive instructions from the field computer in the form of protobuf packets sent over the radio.  See control.{h, c} to see how the motion control code works.  The control code is called from the main() runloop.
