
# Soccer

The 'soccer' program is responsible for receiving data from the vision and referee computers and using the information to strategically command the robots on the field (or in the simulator).

![Screenshot of the 'soccer' program](soccer.png "Soccer")


## Running with the simulator

Start both soccer and the simulator from the terminal:

~~~~~~~~~~~~~~~{.sh}
$ cd run
$ ./simulator &
$ ./soccer -sim
~~~~~~~~~~~~~~~


## Running with real robots

Start soccer:

~~~~~~~~~{.sh}
$ cd run
$ ./soccer
~~~~~~~~~

Also, make sure that the cameras and vision system are on.  If so, robots should show up within the soccer window at their correct locations.


### Manual Control

To run without using the AI/vision system, click the 'Manual' dropdown in the upper right of soccer and select the shell number of the robot you want to control with the joystick.

Here's a diagram of the button layout for joystick control:

![Joystick](joystick-layout.png "Joystick Controls")
