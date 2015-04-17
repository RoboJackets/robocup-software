#pragma once

#include "mbed.h"
#include "cmsis_os.h"
#include "CommLink.hpp"
#include "RTP.hpp"
#include "CC1101-Defines.hpp"

class CC1201 : public CommLink
{
	public:
		CC1201();
	
		CC1201(PinName mosi, PinName miso, PinName sck, PinName cs, PinName intPin = NC);

		std::string modeToStr(uint8_t mode);

		virtual ~CC1201();

		virtual int32_t sendData(uint8_t* buffer, uint8_t size);

		virtual int32_t getData(uint8_t*, uint8_t*);

		virtual void reset(void);
    
		virtual int32_t selfTest(void);
		    
		virtual bool isConnected(void);

		void powerOnReset(void);

		void sendGarbage(void);

		void ready(void);

		/*
		 * lots of reads and writes
		 */
		//TODO move to protected when done testing
		uint8_t strobe(uint8_t addr);

		uint8_t readReg(uint8_t addr);

		void readReg(uint8_t addr, uint8_t* buffer, uint8_t len);

		uint8_t readRegExt(uint8_t addr);

		void readRegExt(uint8_t addr, uint8_t* buffer, uint8_t len);

		void writeReg(uint8_t addr, uint8_t value);

		void writeReg(uint8_t addr, uint8_t* buffer, uint8_t len);

		void writeRegExt(uint8_t addr, uint8_t value);

		void writeRegExt(uint8_t addr, uint8_t* buffer, uint8_t len);

		uint8_t decodeState(uint8_t nopRet);

	protected:

	private:
		uint8_t _chip_version;

		uint8_t mode(void);

		uint8_t status(uint8_t addr);
};

