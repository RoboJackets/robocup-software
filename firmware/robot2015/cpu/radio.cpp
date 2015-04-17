#include <string.h>
#include <stdlib.h>
#include "robot.hpp"
#include "radio.hpp"
#include "CC1201Radio.hpp"

DigitalOut ledThree(LED3);

void initCustomValues(CC1201Config* testRadio)
{
	testRadio->rfendCfg0 = testRadio->rfendCfg0 | TXOFF_MODE_IDLE;
	testRadio->rfendCfg1 = testRadio->rfendCfg1 | RXOFF_MODE_FSTXON;
	testRadio->ecgCfg = testRadio->ecgCfg | 0x0C;
}

void radioThreadHandler(void const* args)
{
	CC1201* testRadio = new CC1201(p5, p6, p7, p9, p8);
	CC1201Config *testRadioConfig = CC1201Config::resetConfiguration(new CC1201Config());

	testRadio->powerOnReset();
	initCustomValues(testRadioConfig);

	while (testRadio->decodeState(testRadio->strobe(CC1201_STROBE_SNOP)) != 0x00)
	{
		testRadio->selfTest();

		if (testRadio->decodeState(testRadio->strobe(CC1201_STROBE_SNOP)) == 0x07)
			testRadio->strobe(CC1201_STROBE_SFTX);
		else if (testRadio->decodeState(testRadio->strobe(CC1201_STROBE_SNOP)) == 0x06)
			testRadio->strobe(CC1201_STROBE_SFRX);
		else
		{
			log(WARN, "radio::radioThreadHandler", 
				"Possible Corrupted State: %02X", 
				testRadio->decodeState(testRadio->strobe(CC1201_STROBE_SNOP))); 
			testRadio->strobe(CC1201_STROBE_SIDLE);
			testRadio->strobe(CC1201_STROBE_SRES);
		}

		Thread::wait(1000);
		testRadio->strobe(CC1201_STROBE_SIDLE);
	}

	CC1201Config::loadConfiguration(testRadioConfig, testRadio);
	//testRadio->strobe(CC1201_STROBE_SRES);
	testRadio->ready();
	

	uint8_t len = 32;
	char* buffer = (char*) malloc(len + 1);
	strcpy(buffer, "Hello, World!");
	
	/*
	while (testRadio->decodeState(testRadio->strobe(CC1201_STROBE_SNOP)) != 0x01)
	{
		printf("NOP: %02X\r\n", testRadio->strobe(CC1201_STROBE_SNOP));
	}
	Thread::wait(100);
	testRadio->strobe(CC1201_STROBE_SIDLE);
	Thread::wait(100);
	*/

	while(true)
	{
		
		//testRadio->strobe(CC1201_STROBE_SFTX);
		//testRadio->strobe(CC1201_STROBE_SFRX);

		testRadio->strobe(CC1201_STROBE_SRX);
		log(INF3, "radio::radioThreadHandler", "NOP Return (decoded): %02X, Marc State: %02X", 
			testRadio->decodeState(testRadio->strobe(CC1201_STROBE_SNOP)),
			testRadio->readRegExt(CC1201EXT_MARCSTATE));
/*
		log(INF3, "radio::radioThreadHandler", "TXFIRST: %02X, TXLAST: %02X, PKTLEN: %02X",
			testRadio->readRegExt(CC1201EXT_TXFIRST),
			testRadio->readRegExt(CC1201EXT_TXLAST),
			testRadio->readReg(CC1201_PKT_LEN));
*/
		fflush(stdout);

		/*
		log(INF2, "radio::radioThreadHandler", "Starting TX");
		testRadio->sendData((uint8_t*) buffer, len);
		testRadio->strobe(CC1201_STROBE_SFTX);
		//testRadio->sendGarbage();		
		log(INF2, "radio::radioThreadHandler", "TX Done.");
		*/

		//Thread::wait(1);
		ledThree = !ledThree;
		Thread::wait(1000);
	}
}
