#include <string.h>
#include <stdlib.h>
#include "robot.hpp"
#include "radio.hpp"
#include "CC1201Radio.hpp"

DigitalOut ledThree(LED3);

void radioThreadHandler(void const* args)
{
	CC1201* testRadio = new CC1201(p5, p6, p7, p9, p8);
	while (testRadio->decodeState(testRadio->strobe(CC1201_STROBE_SNOP)) != 0x00)
	{
		testRadio->selfTest();

		if (testRadio->decodeState(testRadio->strobe(CC1201_STROBE_SNOP)) == 0x07)
			testRadio->strobe(CC1201_STROBE_SFTX);
		else if (testRadio->decodeState(testRadio->strobe(CC1201_STROBE_SNOP)) == 0x06)
			testRadio->strobe(CC1201_STROBE_SFRX);
		
		Thread::wait(1000);
		testRadio->strobe(CC1201_STROBE_SIDLE);
	}

	CC1201Config *testRadioConfig = CC1201Config::resetConfiguration(new CC1201Config());
	CC1201Config::loadConfiguration(testRadioConfig, testRadio);
	//testRadio->strobe(CC1201_STROBE_SRES);
	

	uint8_t len = 32;
	char* buffer = (char*) malloc(len + 1);
	strcpy(buffer, "Hello, World!");
	
	while(true)
	{
		testRadio->strobe(CC1201_STROBE_SFTX);
		testRadio->strobe(CC1201_STROBE_SFRX);

		testRadio->strobe(CC1201_STROBE_SIDLE);
		log(INF3, "radio::radioThreadHandler", "NOP Return (decoded): %02X, Marc State: %02X, Version ID: %02X\r\n", 
			testRadio->decodeState(testRadio->strobe(CC1201_STROBE_SNOP)),
			testRadio->readRegExt(CC1201EXT_MARCSTATE),
			testRadio->readRegExt(CC1201EXT_PARTNUMBER));
		fflush(stdout);

		log(INF2, "radio::radioThreadHandler", "Starting TX");
		testRadio->sendData((uint8_t*) buffer, len);
		log(INF2, "radio::radioThreadHandler", "TX Done.");

		//Thread::wait(1);
		ledThree = !ledThree;
		Thread::wait(1000);
	}
}
