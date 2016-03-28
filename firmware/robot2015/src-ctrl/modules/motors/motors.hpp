#pragma once

// Includes
#include <cstdint>
#include <ctime>
#include <array>
#include <vector>

/* Any math using the delta values for the hall/encoder (we really
 * only care about encoder readings...) is best handled through a task
 * queue where the calculations can be managed on the kernel's scheduler.
 * So we'll leave the MOTOR_MAX_SAMPLES at a minimum in the struct.
 */
#define MOTOR_MAX_SAMPLES 2

extern int start_s;

typedef uint16_t motorVel_t;

struct motorErr_t {
    /*
    Available Flags:
    GVDD_OV, GVDD_UV, PVDD_UV
    OTSD, OTW
    FETHA_OC, FETLA_OC
    FETHB_OC, FETLB_OC
    FETHC_OC, FETLC_OC
    */
    bool hallOK;
    std::array<uint16_t, 2> drvStatus;
};

struct motor_t {
    motorVel_t vel;
    uint16_t hall;
    std::array<uint32_t, MOTOR_MAX_SAMPLES> enc;
    motorErr_t status;
    std::string desc;
};

// TODO(justin): is there a better solution than having a global variable?
extern std::vector<motor_t> global_motors;

void motors_Init();
void motors_show();
int cmd_motors(const std::vector<std::string>&);
int cmd_motors_scroll(const std::vector<std::string>&);
