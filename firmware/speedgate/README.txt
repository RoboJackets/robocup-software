This is a two-beam speed sensor based on the AVR Butterfly.

When power is applied, the speedgate is off.  Press the joystick in any direction to turn it on.
When running, press UP to clear the display or DOWN to power off.
The speedgate will automatically turn off after five minutes without a valid measurement.
The first character in a measurement is an arrow indicating the direction the object was moving.
Pressing RIGHT will toggle the display between speed and time.

Displays:
	------		Measurement not started
	>#####		Speed in mm/s, moving right
	<#####		Speed in mm/s, moving left
	######		Time in microseconds
	FAST		Object too fast to measure (probable jittery input)

If more than one second elapses after one beam is broken without the other beam being broken,
no measurement will be made.  This condition will not reset the auto-power-off timer, so
an object sitting in a beam will not keep the speedgate on.

After a measurement is made, the state of the beams is ignored for one second.
This prevents an object which bounces back into the speedgate from erasing the first measurement.

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
