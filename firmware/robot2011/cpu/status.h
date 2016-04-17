#pragma once

#include <stdint.h>

extern unsigned int robot_id;

// If nonzero, we are on a 2008 base:
//  - Motors run backwards (outside gears)
//  - No encoders
//  - No kicker voltage monitor (use KDONE instead)
//  - No chipper
extern int base2008;

// Failure flags
enum {
    // FPGA did not configure: FPGA bad, SPI flash bad, or SPI flash contents
    // bad
    Fail_FPGA_Config = 0x00000001,

    // FPGA logic failed sanity check: faulty FPGA code
    Fail_FPGA_Logic = 0x00000002,

    // FPGA logic probably works but reports incompatible version: SPI flash not
    // in sync with ARM flash
    Fail_FPGA_Version = 0x00000004,

    // Can't talk to radio at all: bad radio or SPI contention
    Fail_Radio_Interface = 0x00000010,

    // Radio interrupt line is stuck low: probable solder bridge
    Fail_Radio_Int_Low = 0x00000020,

    // Radio interrupt line is stuck high: probable solder bridge
    Fail_Radio_Int_High = 0x00000040,

    // Battery too low for driving
    Fail_Undervoltage = 0x00000100,

    // Supply voltage too high (battery resistance or braking current too high?)
    Fail_Overvoltage = 0x00000200,

    // Motor fuse is blown
    Fail_Fuse = 0x00000400,

    // Ball sensor detector is open
    Fail_Ball_Det_Open = 0x00001000,

    // Ball sensor detector is shorted
    Fail_Ball_Det_Short = 0x00002000,

    // Ball sensor emitter is open
    Fail_Ball_LED_Open = 0x00004000,

    // Ball sensor dazzled
    Fail_Ball_Dazzled = 0x00008000,

    // Gyro/IMU processor
    Fail_Gyro = 0x00010000,

    // Accelerometer
    Fail_Accelerometer = 0x00020000,

    // Kicker voltage monitor not responding
    Fail_Kicker_I2C = 0x00100000,

    // Kicker failed to charge (probably shorted IGBTs)
    Fail_Kicker_Charge = 0x00200000,
};

// Failure categories
#define Fail_FPGA (Fail_FPGA_Config | Fail_FPGA_Logic | Fail_FPGA_Version)
#define Fail_Radio \
    (Fail_Radio_Interface | Fail_Radio_Int_Low | Fail_Radio_Int_High)
#define Fail_Power (Fail_Undervoltage | Fail_Overvoltage | Fail_Fuse)
#define Fail_Ball                                                    \
    (Fail_Ball_Det_Open | Fail_Ball_Det_Short | Fail_Ball_LED_Open | \
     Fail_Ball_Dazzled)
#define Fail_IMU (Fail_Gyro | Fail_Accelerometer)
#define Fail_Kicker (Fail_Kicker_I2C | Fail_Kicker_Charge)

// Motor numbers
// Drive motors 0-3 are labelled M1-M4 on the board.
enum {
    Motor_Back_Left = 0,
    Motor_Front_Left = 1,
    Motor_Front_Right = 2,
    Motor_Back_Right = 3,
    Motor_Dribbler = 4
};

extern unsigned int failures;

// Bits 0-4 correspond to motors 0-4 (see Motor_* above).
extern uint8_t current_motor_faults;

// Latched motor faults.  These are only cleared on reset or with the "fail"
// command.
extern uint8_t motor_faults;

// kicker_status bits
enum {
    Kicker_Charged = 0x01,
    Kicker_Lockout = 0x02,
    Kicker_Charging = 0x04,
    Kicker_Enabled = 0x08,
    Kicker_Override = 0x10,
    Kicker_Chipping = 0x20,
    Kicker_I2C_OK = 0x40
};
