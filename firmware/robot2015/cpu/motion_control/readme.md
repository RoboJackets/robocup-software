
# Robot Motion Control

This folder contains the motion control code that runs in robot firmware, as opposed to the controls that run in 'soccer' on the field laptop.

We use an [LQR (Linear Quadratic Regulator) Controller](https://en.wikipedia.org/wiki/Linear-quadratic_regulator), which calculates optimal control values (motor voltages) using a model of the robot's dynamics and some tunable weights that define what "optimal" means to us.  LQR computes a control gains matrix, **K**, that is multiplied by our velocity error and combined with another "steady state" term to compute motor voltages.  Calculating the gains matrix, **K**, is a fairly expensive operation, so we do it at compile time on the computer rather than at runtime on the MBED, which doesn't have hardware support for floating point math.


## Model

Our model is nonlinear, meaning that it *can't* be written in the form of a linear plant equation:

**dX/dt = A\*X + B*u**

with:

* **X** : the state vector (robot body velocity <x, y, theta>)
* **A**: constant 3x3 matrix
* **B**: constant 3x4 matrix
* **u**: motor voltages

In our case, the fact that the robot can rotate and translate at the same time makes it nonlinear.  The result is that in the above equation, our **A** matrix is dependent on the current radial velocity and is not a constant.  Our plant equation looks like this:

**dX/dt = (A1 + A2\*dPhi/dt)\*X + B*u**

with:

* **A1**: the constant part of **A**
* **A2**: the part of A that depends on the radial velocity
* **dPhi/dt**: the robot's current radial velocity

In order to use LQR, we have to linearize the model at each time step by evaluating for **A** at the current radial velocity.  The effect of this is that unlike regular LQR, there isn't just one value for **K**, there is a value for **K** corresponding to each value of **dPhi/dt**.  To handle this, we decide beforehand the range of radial velocities our robot will encounter (for example -pi rad/s to pi rad/s), then calculate a set number of values of **K** spaced evenly in that range to build a lookup table.  At runtime, we simply find our current radial velocity, then do a lookup into the table to get the value of **K** to use.  See [OfflineLqrController.hpp](OfflineLqrController.hpp) for more info.
