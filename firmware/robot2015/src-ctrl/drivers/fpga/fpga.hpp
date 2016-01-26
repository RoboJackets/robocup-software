#pragma once

#include <mbed.h>
#include <rtos.h>

#include <string>
#include <array>
#include <vector>

#include "pins-ctrl-2015.hpp"

class FPGA {
public:
    static FPGA* Instance();

    bool Init(const std::string& filepath);
    bool isReady();
    uint8_t set_duty_get_enc(uint16_t* duty_cycles, size_t size_dut,
                             uint16_t* enc_deltas, size_t size_enc);
    uint8_t set_duty_cycles(uint16_t* duty_cycles, size_t size);
    uint8_t read_duty_cycles(uint16_t* duty_cycles, size_t size);
    uint8_t read_encs(uint16_t* enc_counts, size_t size);
    uint8_t read_halls(uint8_t* halls, size_t size);
    uint8_t motors_en(bool state);
    uint8_t watchdog_reset();
    bool git_hash(std::vector<uint8_t>&);
    void gate_drivers(std::vector<uint16_t>&);
    bool send_config(const std::string& filepath);

private:
    FPGA() {};

    static bool isInit;
    static FPGA* instance;
    Mutex mutex;

    SPI* spi;
    DigitalOut* cs;
    DigitalInOut* progB;
    DigitalIn* initB;
    DigitalIn* done;
};
