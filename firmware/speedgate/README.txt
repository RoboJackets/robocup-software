This is a two-beam speed sensor based on the AVR Butterfly.

When power is applied, the speedgate is off.  Press the joystick in any direction to turn it on.
When running, press UP to clear the display or DOWN to power off.
Speeds are displayed in mm/s, with a space separating meters from millimeters.
The first character in a measurement is an arrow indicating the direction the object was moving.
Pressing RIGHT will change the display to the time in microseconds of the last reading (mod 1e6).

Displays:
	------		Measurement not started or in progress
	SLOW		Object too slow to measure (probable invalid measurement)
	FAST		Object too fast to measure (probable jittery input)

FIXME - Draw a schematic

The AAA battery pack is connected to VCC_EXT on the Butterfly.  The lithium battery is
not required.  AAA batteries are easier to find and will last longer.

The LEDs are on whenever the display is on.  They draw about 20mA continuously.

Programming:
Get an STK500 or compatible programmer.  The programmer must NOT provide
Vtarget - the AAA batteries will provide this.

On the STK500, remove the VTARGET jumper.  VTARGET may only be connected if
it is providing 3V and the AAA batteries are removed (the LEDs use VCC and
will draw excessive current at 5V).

Connect the programmer to the 6-pin ISP header on the Butterfly.
Pin 1 is toward the inside of the board.
Run 'scons -u speedgate-prog' to program the device.  The programming command
expects to see an STK500 on /dev/ttyS1.

