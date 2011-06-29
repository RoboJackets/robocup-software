#include "kicker.h"
#include "status.h"
#include "fpga.h"
#include "timer.h"

int kicker_test_v1, kicker_test_v2;

static int kicker_test_measure()
{
	// FPGA commands must be sent less than about 227ms apart or the FPGA watchdog timer
	// will turn everything off.
	kicker_charge = 1;
	delay_ms(125);
	fpga_send_commands();
	delay_ms(125);
	fpga_send_commands();
	fpga_read_status();
	return kicker_voltage;
}

// Determines whether the kicker can charge.
// This sets Fail_Kicker_Charge in failures if not.
// The most likely cause is an unplugged kicker or shorted IGBTs.
void kicker_test()
{
	failures &= ~Fail_Kicker_Charge;
	
	if (base2008)
	{
		// Can't tell anything
		kicker_charge = 1;
		return;
	}
	
	// Make sure we can read from the kicker voltage ADC
	fpga_read_status();
	if ((failures & (Fail_Kicker_I2C | Fail_FPGA)) || (kicker_status & Kicker_Override))
	{
		// FPGA doesn't work, ADC doesn't work, or the kicker has been manually disabled.
		// We can't test it, but the charger may still be usable.
		return;
	}
	
	// Clear motor commands
	for (int i = 0; i < 5; ++i)
	{
		motor_out[i] = 0;
	}
	
	// Make two voltage measurements a small time apart.
	// Run the charger just long enough to determine if it works.
	int v1 = kicker_test_measure();
	int v2 = kicker_test_measure();
	
	kicker_test_v1 = v1;
	kicker_test_v2 = v2;
	
	if (	(v1 > 260 || v2 > 260)			// Voltage too high.  Caps are in danger.
		||	(v1 < 10)						// Didn't start charging
		||	(v1 < 80 && (v2 - v1) < 5)		// Voltage was low and didn't increase
	)
	{
		failures |= Fail_Kicker_Charge;
	}
	
	if (failures & Fail_Kicker_Charge)
	{
		// Turn off charging
		kicker_charge = 0;
	}
	
	// Leave kicker_charge on so we can use the kicker
}

void kicker_monitor()
{
	// This is called after fpga_read_status() so this function may change kicker_status.
	
	if (base2008)
	{
		// The only feedback we have is KDONE, which is already set
		//FIXME - Charging timeout
	} else {
		// Use measured voltage instead of KDONE
		if (kicker_voltage >= 230)
		{
			kicker_status |= Kicker_Charged;
		} else {
			kicker_status &= ~Kicker_Charged;
		}
		
		//FIXME - Look for charging failure
	}
}
