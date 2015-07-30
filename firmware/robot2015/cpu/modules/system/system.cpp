#include "system.hpp"

bool System::isInit = false;

/**
 *
 */
System::System(void)
{
	// do stuff
}


/**
 *
 */
System::~System(void)
{
	isInit = false;
}


/**
 * [System::Init Setup the FPGA interface]
 * @return  [The initialization error code.]
 */
void System::Launch(void)
{
	if (isInit == true)
		return;

	if (COMPETITION_DEPLOY == false)
		Thread console_task(Task_SerialConsole);

	isInit = true;
}
