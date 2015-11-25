#pragma once

// Includes
#include <cstdint>
#include <array>
#include <vector>

/* Any math using the delta values for the hall/encoder (we really
 * only care about encoder readings...) is best handled through a task
 * queue where the calculations can be managed on the kernel's scheduler.
 * So we'll leave the MOTOR_MAX_SAMPLES at a minimum in the struct.
 */
#define MOTOR_MAX_SAMPLES 2

typedef uint16_t motorVel_t;

struct motorErr_t {
    /*
    Available Flags:
    =================
    GVDD_OV
    GVDD_UV
    PVDD_UV
    OTSD
    OTW
    FETHA_OC
    FETLA_OC
    FETHB_OC
    FETLB_OC
    FETHC_OC
    FETLC_OC
    */
    bool encOK;
    bool hallOK;
    std::array<uint16_t, 2> drvStatus;
};

struct motor_t {
    motorVel_t targetVel;
    motorVel_t adjVel;
    uint16_t hallCount;
    std::array<uint32_t, MOTOR_MAX_SAMPLES> encCounts;
    motorErr_t status;
    std::string desc;
};

void motors_Init();
std::ostream& cmd_motors_print(std::ostream&, const motor_t&);
std::ostream& cmd_motors(std::ostream&, const std::vector<std::string>&);
std::ostream& cmd_motors_scroll(std::ostream&, const std::vector<std::string>&);
