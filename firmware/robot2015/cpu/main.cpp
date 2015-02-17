#include "mbed.h"
#include "console.hpp"
#include "commands.hpp"
#include "IOExpander.h";

Ticker lifeLight;
DigitalOut ledOne(LED1);
DigitalOut ledTwo(LED2);

/**
 * timer interrupt based light flicker
 */
void imAlive()
{
	ledOne = !ledOne;
}

/**
 * system entry point
 */
int main() 
{
	lifeLight.attach(&imAlive, 0.25);
	initConsole();

    	for (;;)
	{
		/*
		 * check console communications, currently does nothing
		 * then execute any active iterative command
 		 */
		conComCheck();
		//execute any active iterative command
		executeIterativeCommand();

		/*
		 * check if a system stop is requested
		 */
		if (isSysStopReq() == true)
		{
			break;
		}

		//main loop heartbeat
		wait(0.1);
		ledTwo = !ledTwo;
    	}

	//clear light for main loop (shows its complete)
	ledTwo = false;
}

