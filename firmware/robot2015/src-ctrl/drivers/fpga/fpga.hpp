#pragma once

#include <mbed.h>
#include <string>
#include <array>

#include "pins-ctrl-2015.hpp"

class FPGA
{
public:
    FPGA(void) : spi(RJ_SPI_BUS), cs(RJ_FPGA_nCS, 1), progB(RJ_FPGA_PROG_B, PIN_OUTPUT, OpenDrain, 1), initB(RJ_FPGA_INIT_B), done(RJ_FPGA_DONE) { };
    FPGA(PinName _mosi, PinName _miso, PinName _sck, PinName _cs, PinName _progB, PinName _initB, PinName _done);
    ~FPGA(void);

    bool Init(const std::string& filepath);
    uint8_t read_halls(uint8_t* halls, size_t size);
    uint8_t read_duty_cycles(uint16_t* duty_cycles, size_t size);
    uint32_t git_hash(void);
    uint8_t set_duty_get_enc(uint16_t* duty_cycles, size_t size_dut, uint16_t* enc_deltas, size_t size_enc);
    uint8_t motors_en(bool state);

private:
    static bool     isInit;

    bool send_config(const std::string& filepath);

    SPI             spi;
    DigitalOut      cs;
    DigitalInOut    progB;
    DigitalIn       initB, done;
};
