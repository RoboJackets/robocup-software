#include <string.h>
#include <stdlib.h>
#include "robot.hpp"
#include "radio.hpp"
#include "CC1201Radio.hpp"

DigitalOut ledThree(LED3);
DigitalOut ledFour(LED4);
InterruptIn rxInt(p8);

void initCustomValues(CC1201Config* testRadio)
{
	testRadio->partnumber = 0x21;
	testRadio->partversion = 0x11;
	testRadio->rfendCfg0 = testRadio->rfendCfg0 | TXOFF_MODE_RX;
	testRadio->rfendCfg1 = testRadio->rfendCfg1 | RXOFF_MODE_TX;
	testRadio->ecgCfg = testRadio->ecgCfg | 0x0C;
}

void intTest(void)
{
	ledFour = !ledFour;
}

void radioThreadHandler(void const* args)
{
	rxInt.rise(&intTest);

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
	if (CC1201Config::verifyConfiguration(testRadioConfig, testRadio) == false)
	{
		while (!CC1201Config::configurationFaults.empty())
		{
			log(WARN, "radio::radioThreadHandler", CC1201Config::configurationFaults.front().c_str());
			CC1201Config::configurationFaults.pop();
		}
	}
	else
	{
		log(INF1, "radio::radioThreadHandler", "radio configuration confirmed.");
	}
	testRadio->ready();
	

	uint8_t len = 32;
	uint8_t dataLen = 13;
	char* buffer = (char*) malloc(len);
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

	//testRadio->strobe(CC1201_STROBE_SRX);
	testRadio->strobe(CC1201_STROBE_SIDLE);
	while(true)
	{
		//testRadio->strobe(CC1201_STROBE_SIDLE);
		//Thread::wait(1000);
		//testRadio->strobe(CC1201_STROBE_SCAL);
		//Thread::wait(1000);
		//testRadio->strobe(CC1201_STROBE_SRX);

		/*log(INF3, "radio::radioThreadHandler", "NOP Return (decoded): %02X, Marc State: %02X", 
			testRadio->decodeState(testRadio->strobe(CC1201_STROBE_SNOP)),
			testRadio->readRegExt(CC1201EXT_MARCSTATE));*/
/*
		log(INF3, "radio::radioThreadHandler", "TXFIRST: %02X, TXLAST: %02X, PKTLEN: %02X",
			testRadio->readRegExt(CC1201EXT_TXFIRST),
			testRadio->readRegExt(CC1201EXT_TXLAST),
			testRadio->readReg(CC1201_PKT_LEN));
*/

		//log(INF2, "radio::radioThreadHandler", "Starting TX");
		//testRadio->sendData((uint8_t*) buffer, dataLen);
		//testRadio->strobe(CC1201_STROBE_SFTX);
		//testRadio->sendGarbage();		
		//log(INF2, "radio::radioThreadHandler", "TX Done.");

		if (CC1201Config::verifyConfiguration(testRadioConfig, testRadio) == false)
		{
			log(WARN, "radio::radioThreadHandler", "radio configuration corrupted");
			while (!CC1201Config::configurationFaults.empty())
			{
				log(WARN, "radio::radioThreadHandler", CC1201Config::configurationFaults.front().c_str());
				CC1201Config::configurationFaults.pop();
			}
			log(INF2, "radio::radioThreadHandler", "radio configuration dumped");
		}
		else
		{
			log(INF1, "radio::radioThreadHandler", "radio configuration confirmed.");
		}

		//Thread::wait(1);
		ledThree = !ledThree;
		Thread::wait(5000);
	}
}
