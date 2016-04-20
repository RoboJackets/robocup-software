#pragma once

#include <mbed.h>
#include <rtos.h>

#include <mcp23017.hpp>
#include <io-expander.hpp>
#include <helper-funcs.hpp>
#include <logger.hpp>
#include <Console.hpp>

#include "pins-ctrl-2015.hpp"
#include "task-signals.hpp"

/**
 * A DigitalInOut class meant to replicate basic functionality of the
 * mBed DigitalOut and DigitalIn on the io-expander.
 */
class RobotExpanderInputs {
public:
    /// Other constructors for creating objects for pinouts
    RobotExpanderInputs(MCP23017* mcp, PinName interruptPin)
        : interruptThread_(&RobotExpanderInputs::ThreadHelper, this, osPriorityBelowNormal,
                    0.33 * DEFAULT_STACK_SIZE),
          mcp23017_(mcp),
          interrupt_(interruptPin) {
            interrupt_.fall(this, &RobotExpanderInputs::ISR);
            interrupt_.mode(PullUp);
    }

    uint8_t RotarySelector() {
        uint8_t reading = 0;
        const std::array<MCP23017::ExpPinName, 4> kPins = {
            RJ_HEX_SWITCH_BIT0, RJ_HEX_SWITCH_BIT1, RJ_HEX_SWITCH_BIT2,
            RJ_HEX_SWITCH_BIT3};

        for (size_t i = 0; i < kPins.size(); i++)
            reading |= mcp23017_->readPin(kPins[i]) << i;
        return reading;
    }

    uint8_t DipSwitches() {
        uint8_t reading = 0;
        const std::array<MCP23017::ExpPinName, 3> kPins = {
            RJ_DIP_SWITCH_1, RJ_DIP_SWITCH_2, RJ_DIP_SWITCH_3};

        for (size_t i = 0; i < kPins.size(); i++)
            reading |= mcp23017_->readPin(kPins[i]) << i;
        return reading;
    }

    bool PushButton() { return mcp23017_->readPin(RJ_PUSHBUTTON); }

    /**
     * Gives the rotarySelector reading in string standard format.
    */
    std::string RotarySelectorString() {
        char buf[2];
        sprintf(buf, "%02u", RotarySelector());
        return std::string(buf);
    }

    std::string DipSwitchesString() {
        uint8_t state = DipSwitches();
        std::string buf(state & 0x01 ? "ON" : "OFF");
        for (size_t i = 1; i <= 2; i++)
            buf += (state & (1 << i)) ? ",ON" : ",OFF";
        return buf;
    }

    std::string PushButtonString() {
        return std::string(PushButton() ? "ON" : "OFF");
    }

    void RefreshEnvironment() {
        uint8_t robot_id = RotarySelector();
        uint8_t dip_switches = DipSwitches();
        bool push_btn = PushButton();

        // TODO: get these values to where they need to go
    }

    void Task_InputHandler() {
        while (true) {
            // refresh system variables
            RefreshEnvironment();
            /* wait until signaled through RTOS, set a threshold delay for
             * bounching */
            Thread::wait(100);
            Thread::signal_wait(MAIN_TASK_CONTINUE, osWaitForever);
        };
    }

    void ISR() { interruptThread_.signal_set(MAIN_TASK_CONTINUE); }

    static void ThreadHelper(const void* args) {
        RobotExpanderInputs* obj = (RobotExpanderInputs*)args;
        obj->Task_InputHandler();
    }

private:
    Thread interruptThread_;
    MCP23017* mcp23017_;
    InterruptIn interrupt_;
};
