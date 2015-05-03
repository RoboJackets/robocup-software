#include <string.h>
#include <stdlib.h>
#include "robot.hpp"
#include "radio.hpp"
#include "CC1201Radio.hpp"

DigitalOut ledThree3(LED3);
DigitalOut ledFour4(LED4);
InterruptIn rxInt(p9);

void initCustomValues(CC1201Config* testRadio)
{
	//testRadio->partnumber = 0x21;
	//testRadio->partversion = 0x11;
	testRadio->rfendCfg0 = 0x00;
	testRadio->rfendCfg1 = 0x0F;
	testRadio->rfendCfg0 |= TXOFF_MODE_RX;
	testRadio->rfendCfg1 |= RXOFF_MODE_IDLE;
	//testRadio->ecgCfg = testRadio->ecgCfg | 0x0C;
}

void intTest(void)
{
	ledFour4 = !ledFour4;
}

void radioThreadHandler(void const* args)
{
	CC1201* testRadio = new CC1201(RJ_SPI_BUS, p8);
	CC1201Config *testRadioConfig = CC1201Config::resetConfiguration(new CC1201Config());	// The Power On Reset takes care of clearing registers in the constructor
	initCustomValues(testRadioConfig);
	CC1201Config::loadConfiguration(testRadioConfig, testRadio);
	
	while (testRadio->mode() != 0x01)
	{
		uint8_t current_state = testRadio->decodeState(testRadio->strobe(CC1201_STROBE_SIDLE));
		 //= testRadio->decodeState(testRadio->status());	

		if ( (current_state == 0x07) | (current_state == 0x06) )
		{
			testRadio->strobe(CC1201_STROBE_SIDLE);
			testRadio->strobe(CC1201_STROBE_SFTX);
			testRadio->strobe(CC1201_STROBE_SFRX);
		}
		else if (current_state != 0x00)	// If not IDLE
		{
			log(
				WARN,
				"radio::radioThreadHandler", 
				"Possible Corrupted State: %02X", 
				testRadio->mode()
			);
		}

		Thread::wait(50);
	}

	if (CC1201Config::verifyConfiguration(testRadioConfig, testRadio) == false)
	{
        ledFour4 = !ledFour4;
        for (;;);

		while (!CC1201Config::configurationFaults.empty())
		{
			log(WARN, "radio::radioThreadHandler",
				CC1201Config::configurationFaults.front().c_str());

			CC1201Config::configurationFaults.pop();
		}
	}
	else
	{
		log(INF1, "radio::radioThreadHandler", "radio configuration confirmed.");
	}


	uint8_t buf_len = 32;
	uint8_t dataLen = 5;
	uint8_t buffer[] = { 1, 2, 3, 4, 5 };
	//strcpy(buffer, "Hello, World!");


	while(true)
	{
		testRadio->strobe(CC1201_STROBE_SCAL);
		Thread::wait(100);
		log(INF1, "radio.cpp", "Sending %u bytes", dataLen);
		testRadio->sendData(buffer, dataLen);
		ledThree3 = !ledThree3;
		Thread::wait(1000);

		/*
		testRadio->strobe(CC1201_STROBE_SIDLE);
		while (testRadio->decodeState(testRadio->status()) != 0x00)
		Thread::wait(100);
		testRadio->strobe(CC1201_STROBE_SRX);
		while (testRadio->decodeState(testRadio->status()) != 0x01)
		*/
	}
}
