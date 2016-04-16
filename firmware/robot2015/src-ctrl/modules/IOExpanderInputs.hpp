#pragma once

#include <mbed.h>
#include <rtos.h>

#include <mcp23017.hpp>
#include <io-expander.hpp>
#include <RotarySelector.hpp>
#include <helper-funcs.hpp>
#include <logger.hpp>
#include <Console.hpp>

#include "pins-ctrl-2015.hpp"
#include "task-signals.hpp"

/**
 * A DigitalInOut class meant to replicate basic functionality of the
 * mBed DigitalOut and DigitalIn on the io-expander.
 */
class RobotInputs {
public:
    /// Other constructors for creating objects for pinouts
    RobotInputs(MCP23017* mcp, PinName interruptPin)
        : _rxThread(&RobotInputs::ThreadHelper, this, osPriorityBelowNormal,
                    0.33 * DEFAULT_STACK_SIZE),
          _mcp23017(mcp),
          _interrupt(interruptPin) {
        _interrupt.fall(this, &RobotInputs::ISR);
        _interrupt.mode(PullUp);
        Thread::wait(10);
    }

    uint8_t rotarySelector() {
        uint8_t reading = 0;
        const std::array<MCP23017::ExpPinName, 4> pins = {
            RJ_HEX_SWITCH_BIT0, RJ_HEX_SWITCH_BIT1, RJ_HEX_SWITCH_BIT2,
            RJ_HEX_SWITCH_BIT3};

        for (size_t i = 0; i < pins.size(); i++)
            reading |= _mcp23017->readPin(pins[i]) << i;
        return reading;
    }

    /**
     * Gives the rotarySelector reading in string standard format.
    */
    std::string rotarySelectorString() {
        char buf[2];
        sprintf(buf, "%02u", rotarySelector());
        return std::string(buf);
    }

    uint8_t dipSwitches() {
        uint8_t reading = 0;
        const std::array<MCP23017::ExpPinName, 3> pins = {
            RJ_DIP_SWITCH_1, RJ_DIP_SWITCH_2, RJ_DIP_SWITCH_3};

        for (size_t i = 0; i < pins.size(); i++)
            reading |= _mcp23017->readPin(pins[i]) << i;
        return reading;
    }

    std::string dipSwitchesString() {
        uint8_t state = dipSwitches();
        std::string buf(state & 0x01 ? "ON" : "OFF");
        for (size_t i = 1; i <= 2; i++)
            buf += (state & (1 << i)) ? ",ON" : ",OFF";
        return buf;
    }

    bool pushButton() { return _mcp23017->readPin(RJ_PUSHBUTTON); }

    std::string pushButtonString() {
        return std::string(pushButton() ? "ON" : "OFF");
    }

    void resetInterrupts() {
        for (int reg_addr = 0; reg_addr <= MCP23017::OLAT; reg_addr += 2)
            _mcp23017->readRegister(static_cast<MCP23017::Register>(reg_addr));
    }

    void refreshEnvironment() {
        std::string id(rotarySelectorString());
        Thread::wait(1);
        std::string sw(dipSwitchesString());
        Thread::wait(1);
        std::string btn(pushButtonString());
        rj_putenv("ROBOT_ID=" + id);
        rj_putenv("ROBOT_DIP=" + sw);
        rj_putenv("ROBOT_BTN=" + btn);
    }

    void Task_InputHandler() {
        while (true) {
            // refresh system variables
            refreshEnvironment();
            /* wait until signaled through RTOS, set a threshold delay for
             * bounching */
            Thread::wait(100);
            Thread::signal_wait(MAIN_TASK_CONTINUE, osWaitForever);
        };
    }

    void ISR() { _rxThread.signal_set(MAIN_TASK_CONTINUE); }

    static void ThreadHelper(const void* args) {
        RobotInputs* obj = (RobotInputs*)args;
        obj->Task_InputHandler();
    }

private:
    Thread _rxThread;
    MCP23017* _mcp23017;
    InterruptIn _interrupt;
};
